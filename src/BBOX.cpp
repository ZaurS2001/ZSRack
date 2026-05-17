#include "plugin.hpp"
#include <system.hpp>
#include <vector>
#include <string>
#include <cmath>
#include <osdialog.h>
#include <cstdint>

void copyWavFile(const std::string& src, const std::string& dst) {
    if (src == dst || src.empty()) return;
    FILE* s = fopen(src.c_str(), "rb");
    if (!s) return;
    FILE* d = fopen(dst.c_str(), "wb");
    if (!d) { fclose(s); return; }
    char buffer[8192];
    size_t bytes;
    while ((bytes = fread(buffer, 1, sizeof(buffer), s)) > 0) {
        fwrite(buffer, 1, bytes, d);
    }
    fclose(s);
    fclose(d);
}

// Robust WAV loader supporting 16-bit, 24-bit, and 32-bit Float PCM
struct SampleData {
    std::vector<float> data;
    std::string name;
    
    void loadWav(const std::string& path) {
        data.clear();
        FILE* f = fopen(path.c_str(), "rb");
        if (!f) return;
        
        char magic[4];
        if (fread(magic, 1, 4, f) != 4 || strncmp(magic, "RIFF", 4) != 0) { fclose(f); return; }
        uint32_t fileSize; if (fread(&fileSize, 4, 1, f) != 1) { fclose(f); return; }
        if (fread(magic, 1, 4, f) != 4 || strncmp(magic, "WAVE", 4) != 0) { fclose(f); return; }

        int channels = 1;
        int bitsPerSample = 16;
        bool fmtFound = false;

        while (fread(magic, 1, 4, f) == 4) {
            uint32_t chunkSize;
            if (fread(&chunkSize, 4, 1, f) != 1) break;

            if (strncmp(magic, "fmt ", 4) == 0) {
                uint16_t audioFormat, numChannels;
                uint32_t sampleRate, byteRate;
                uint16_t blockAlign, bps;
                if (fread(&audioFormat, 2, 1, f) != 1) break;
                if (fread(&numChannels, 2, 1, f) != 1) break;
                if (fread(&sampleRate, 4, 1, f) != 1) break;
                if (fread(&byteRate, 4, 1, f) != 1) break;
                if (fread(&blockAlign, 2, 1, f) != 1) break;
                if (fread(&bps, 2, 1, f) != 1) break;
                
                channels = numChannels;
                bitsPerSample = bps;
                fmtFound = true;
                
                if (chunkSize > 16) fseek(f, chunkSize - 16 + (chunkSize % 2), SEEK_CUR);
                else if (chunkSize % 2) fseek(f, 1, SEEK_CUR);

            } else if (strncmp(magic, "data", 4) == 0) {
                if (!fmtFound) break; 
                
                long dataBytes = chunkSize;
                if (bitsPerSample == 16) {
                    std::vector<int16_t> raw(dataBytes / 2);
                    fread(raw.data(), 2, raw.size(), f);
                    data.resize(raw.size() / channels);
                    for (size_t i = 0; i < data.size(); i++) data[i] = raw[i * channels] / 32768.0f; 
                } else if (bitsPerSample == 24) {
                    std::vector<uint8_t> raw(dataBytes);
                    fread(raw.data(), 1, dataBytes, f);
                    data.resize((dataBytes / 3) / channels);
                    for (size_t i = 0; i < data.size(); i++) {
                        size_t off = i * channels * 3;
                        int32_t sample = (raw[off] << 8) | (raw[off+1] << 16) | (raw[off+2] << 24);
                        data[i] = (sample >> 8) / 8388608.0f;
                    }
                } else if (bitsPerSample == 32) {
                    std::vector<float> raw(dataBytes / 4);
                    fread(raw.data(), 4, raw.size(), f);
                    data.resize(raw.size() / channels);
                    for (size_t i = 0; i < data.size(); i++) data[i] = raw[i * channels];
                }
                break;
            } else {
                fseek(f, chunkSize + (chunkSize % 2), SEEK_CUR);
            }
        }
        fclose(f);
        
        size_t lastSlash = path.find_last_of("/\\");
        name = (lastSlash == std::string::npos) ? path : path.substr(lastSlash + 1);
    }
};

struct BBOX : Module {
    enum ParamId {
        GLOBAL_MUTE_PARAM, GLOBAL_VOL_PARAM,
        VEL_PARAM, TUNE_PARAM = VEL_PARAM + 8, PUNCH_PARAM = TUNE_PARAM + 8,
        DECAY_PARAM = PUNCH_PARAM + 8, PAN_PARAM = DECAY_PARAM + 8, VOL_PARAM = PAN_PARAM + 8,
        PARAMS_LEN = VOL_PARAM + 8
    };
    enum InputId {
        CLK_INPUT, GATE_INPUT,
        VEL_INPUT = GATE_INPUT + 8, TUNE_INPUT = VEL_INPUT + 8, PUNCH_INPUT = TUNE_INPUT + 8,
        DECAY_INPUT = PUNCH_INPUT + 8, PAN_INPUT = DECAY_INPUT + 8, VOL_INPUT = PAN_INPUT + 8,
        INPUTS_LEN = VOL_INPUT + 8
    };
    enum OutputId {
        MIX_L_OUTPUT, MIX_R_OUTPUT, TRK_OUTPUT,
        OUTPUTS_LEN = TRK_OUTPUT + 8
    };

    SampleData samples[8];
    std::string samplePaths[8];
    std::string kitName = "Default Kit";
    
    bool isMuted[8] = {false};
    bool isSolo[8] = {false};
    bool forceUnmute[8] = {false};
    int muteGroups[8] = {0}; 

    bool playing[8] = {false};
    float phase[8] = {0.f};
    int envState[8] = {0}; 
    float envVal[8] = {0.f};
    
    dsp::SchmittTrigger globalClk;
    dsp::SchmittTrigger trackTriggers[8];

    BBOX() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN);
        configParam(GLOBAL_MUTE_PARAM, 0.f, 1.f, 0.f, "Global Mute");
        configParam(GLOBAL_VOL_PARAM, 0.f, 2.f, 1.f, "Global Volume");
        configOutput(MIX_L_OUTPUT, "Main Mix L");
        configOutput(MIX_R_OUTPUT, "Main Mix R");

        for (int i = 0; i < 8; i++) {
            configInput(GATE_INPUT + i, string::f("Track %d Gate", i+1));
            configInput(VEL_INPUT + i, string::f("Track %d Velocity CV", i+1));
            configInput(TUNE_INPUT + i, string::f("Track %d Tune CV", i+1));
            configInput(PUNCH_INPUT + i, string::f("Track %d Punch CV", i+1));
            configInput(DECAY_INPUT + i, string::f("Track %d Decay CV", i+1));
            configInput(PAN_INPUT + i, string::f("Track %d Pan CV", i+1));
            configInput(VOL_INPUT + i, string::f("Track %d Volume CV", i+1));

            configParam(VEL_PARAM + i, 0.f, 127.f, 100.f, string::f("Track %d Velocity", i+1));
            configParam(TUNE_PARAM + i, -6.f, 6.f, 0.f, string::f("Track %d Tune", i+1), " st");
            configParam(PUNCH_PARAM + i, 0.f, 127.f, 10.f, string::f("Track %d Punch", i+1));
            configParam(DECAY_PARAM + i, 0.f, 127.f, 64.f, string::f("Track %d Decay", i+1));
            configParam(PAN_PARAM + i, -1.f, 1.f, 0.f, string::f("Track %d Pan", i+1));
            configParam(VOL_PARAM + i, 0.f, 2.f, 1.f, string::f("Track %d Volume", i+1));
            
            configOutput(TRK_OUTPUT + i, string::f("Track %d Output", i+1));
        }
    }

    void process(const ProcessArgs& args) override {
        bool globalMuted = params[GLOBAL_MUTE_PARAM].getValue() > 0.5f;
        float globalVol = params[GLOBAL_VOL_PARAM].getValue();
        
        bool anySolo = false;
        for (int i = 0; i < 8; i++) if (isSolo[i]) anySolo = true;
        
        bool clkTriggered = globalClk.process(inputs[CLK_INPUT].getVoltage());
        float mixL = 0.f; float mixR = 0.f;

        for (int i = 0; i < 8; i++) {
            bool trg = false;
            
            if (inputs[GATE_INPUT + i].isConnected()) {
                if (trackTriggers[i].process(inputs[GATE_INPUT + i].getVoltage())) trg = true;
            } else if (clkTriggered) {
                trg = true;
            }

            float velVoltage = inputs[VEL_INPUT + i].isConnected() ? inputs[VEL_INPUT + i].getVoltage() : 10.f;

            bool canPlay = false;
            if (anySolo) canPlay = isSolo[i] || forceUnmute[i];
            else { canPlay = !isMuted[i]; forceUnmute[i] = false; }
            if (globalMuted) canPlay = false;

            if (trg && canPlay && !samples[i].data.empty()) {
                playing[i] = true; phase[i] = 0.f; envState[i] = 1; 
                
                for (int j = 0; j < 8; j++) {
                    if (i != j && playing[j] && (muteGroups[i] & muteGroups[j])) playing[j] = false; 
                }
            }

            float outSample = 0.f;

            if (playing[i] && !samples[i].data.empty()) {
                float tuneCV = inputs[TUNE_INPUT + i].getVoltage();
                float totalTune = params[TUNE_PARAM + i].getValue() + tuneCV;
                float rate = std::pow(2.f, totalTune / 12.f);
                
                float punchCV = clamp(inputs[PUNCH_INPUT + i].getVoltage() / 10.f, 0.f, 1.f);
                float attackTime = std::pow(params[PUNCH_PARAM + i].getValue() / 127.f + punchCV, 2.f) * 0.5f + 0.001f;
                
                float decayCV = clamp(inputs[DECAY_INPUT + i].getVoltage() / 10.f, 0.f, 1.f);
                float decayTime = std::pow(params[DECAY_PARAM + i].getValue() / 127.f + decayCV, 2.f) * 5.0f + 0.01f;

                if (envState[i] == 1) { 
                    envVal[i] += 1.f / (attackTime * args.sampleRate);
                    if (envVal[i] >= 1.f) { envVal[i] = 1.f; envState[i] = 2; }
                } else if (envState[i] == 2) { 
                    envVal[i] -= 1.f / (decayTime * args.sampleRate);
                    if (envVal[i] <= 0.f) { envVal[i] = 0.f; envState[i] = 0; playing[i] = false; }
                }

                int idx = (int)phase[i];
                if (idx >= 0 && (size_t)idx < samples[i].data.size() - 1) {
                    float frac = phase[i] - idx;
                    float raw = samples[i].data[idx] * (1.f - frac) + samples[i].data[idx + 1] * frac;
                    
                    float velNorm = clamp(params[VEL_PARAM + i].getValue() / 127.f * (velVoltage / 10.f), 0.f, 1.f);
                    float volNorm = clamp(params[VOL_PARAM + i].getValue() + inputs[VOL_INPUT + i].getVoltage() / 10.f, 0.f, 2.f);
                    
                    outSample = raw * envVal[i] * velNorm * volNorm;
                    phase[i] += rate;
                } else playing[i] = false;
            }
            
            outputs[TRK_OUTPUT + i].setVoltage(outSample * 5.f);
            
            float panMod = clamp(params[PAN_PARAM + i].getValue() + inputs[PAN_INPUT + i].getVoltage() / 5.f, -1.f, 1.f);
            float panAngle = (panMod + 1.f) * 0.5f * M_PI * 0.5f;
            mixL += outSample * std::cos(panAngle) * globalVol * 5.f;
            mixR += outSample * std::sin(panAngle) * globalVol * 5.f;
        }

        outputs[MIX_L_OUTPUT].setVoltage(mixL);
        outputs[MIX_R_OUTPUT].setVoltage(mixR);
    }

    void loadSample(int track, std::string path) {
        if (track >= 0 && track < 8) {
            samples[track].loadWav(path);
            samplePaths[track] = path;
        }
    }

    void clearSample(int track) {
        if (track >= 0 && track < 8) {
            samples[track].data.clear();
            samples[track].name = "";
            samplePaths[track] = "";
            playing[track] = false;
        }
    }

    void clearAllSamples() {
        for (int i = 0; i < 8; i++) {
            clearSample(i);
        }
    }

    json_t* dataToJson() override {
        json_t* root = json_object();
        json_object_set_new(root, "kitName", json_string(kitName.c_str()));
        json_t* paths = json_array();
        json_t* mgroups = json_array();
        for (int i = 0; i < 8; i++) {
            json_array_append_new(paths, json_string(samplePaths[i].c_str()));
            json_array_append_new(mgroups, json_integer(muteGroups[i]));
        }
        json_object_set_new(root, "paths", paths);
        json_object_set_new(root, "muteGroups", mgroups);
        return root;
    }

    void dataFromJson(json_t* root) override {
        json_t* jname = json_object_get(root, "kitName");
        if (jname) kitName = json_string_value(jname);
        json_t* paths = json_object_get(root, "paths");
        json_t* mgroups = json_object_get(root, "muteGroups");
        for (int i = 0; i < 8; i++) {
            if (paths) {
                json_t* jp = json_array_get(paths, i);
                if (jp && std::string(json_string_value(jp)) != "") loadSample(i, json_string_value(jp));
            }
            if (mgroups) {
                json_t* jm = json_array_get(mgroups, i);
                if (jm) muteGroups[i] = json_integer_value(jm);
            }
        }
    }
    json_t* kitToJson(const std::string& kitDir) {
        json_t* root = json_object();
        json_object_set_new(root, "kitName", json_string(kitName.c_str()));
        
        json_t* paths = json_array();
        json_t* mgroups = json_array();
        json_t* paramsArray = json_array();
        
        for (int i = 0; i < 8; i++) {
            std::string newPath = "";
            if (!samplePaths[i].empty()) {
                size_t lastSlash = samplePaths[i].find_last_of("/\\");
                std::string filename = (lastSlash == std::string::npos) ? samplePaths[i] : samplePaths[i].substr(lastSlash + 1);
                newPath = kitDir + "/" + filename;
                
                // Copy the physical sample into the kit folder
                copyWavFile(samplePaths[i], newPath);
                
                // Keep track of the bundled path so standard Rack patch saves also use it
                samplePaths[i] = newPath; 
            }
            json_array_append_new(paths, json_string(newPath.c_str()));
            json_array_append_new(mgroups, json_integer(muteGroups[i]));
            
            // Serialize Parameters
            json_t* trkParams = json_object();
            json_object_set_new(trkParams, "vel", json_real(params[VEL_PARAM + i].getValue()));
            json_object_set_new(trkParams, "tune", json_real(params[TUNE_PARAM + i].getValue()));
            json_object_set_new(trkParams, "punch", json_real(params[PUNCH_PARAM + i].getValue()));
            json_object_set_new(trkParams, "decay", json_real(params[DECAY_PARAM + i].getValue()));
            json_object_set_new(trkParams, "pan", json_real(params[PAN_PARAM + i].getValue()));
            json_object_set_new(trkParams, "vol", json_real(params[VOL_PARAM + i].getValue()));
            json_array_append_new(paramsArray, trkParams);
        }
        json_object_set_new(root, "paths", paths);
        json_object_set_new(root, "muteGroups", mgroups);
        json_object_set_new(root, "params", paramsArray);
        
        return root;
    }

    void kitFromJson(json_t* root) {
        json_t* jname = json_object_get(root, "kitName");
        if (jname) kitName = json_string_value(jname);
        
        json_t* paths = json_object_get(root, "paths");
        json_t* mgroups = json_object_get(root, "muteGroups");
        json_t* paramsArray = json_object_get(root, "params");
        
        for (int i = 0; i < 8; i++) {
            if (paths) {
                json_t* jp = json_array_get(paths, i);
                if (jp && std::string(json_string_value(jp)) != "") {
                    loadSample(i, json_string_value(jp));
                } else {
                    samples[i].data.clear();
                    samples[i].name = "";
                    samplePaths[i] = "";
                }
            }
            if (mgroups) {
                json_t* jm = json_array_get(mgroups, i);
                if (jm) muteGroups[i] = json_integer_value(jm);
            }
            if (paramsArray) {
                json_t* trkParams = json_array_get(paramsArray, i);
                if (trkParams) {
                    json_t* v = json_object_get(trkParams, "vel"); if(v) params[VEL_PARAM + i].setValue(json_real_value(v));
                    json_t* t = json_object_get(trkParams, "tune"); if(t) params[TUNE_PARAM + i].setValue(json_real_value(t));
                    json_t* p = json_object_get(trkParams, "punch"); if(p) params[PUNCH_PARAM + i].setValue(json_real_value(p));
                    json_t* d = json_object_get(trkParams, "decay"); if(d) params[DECAY_PARAM + i].setValue(json_real_value(d));
                    json_t* pan = json_object_get(trkParams, "pan"); if(pan) params[PAN_PARAM + i].setValue(json_real_value(pan));
                    json_t* vol = json_object_get(trkParams, "vol"); if(vol) params[VOL_PARAM + i].setValue(json_real_value(vol));
                }
            }
        }
    }

    void saveKit() {
        std::string defaultDir = asset::plugin(pluginInstance, "kits");
        system::createDirectory(defaultDir); // Ensure the root kits folder exists

        char* path = osdialog_file(OSDIALOG_SAVE, defaultDir.c_str(), "kit.bbox", NULL);
        if (path) {
            std::string kitPath = path;
            
            // Create a dedicated folder for this kit based on the name inputted
            std::string folderName = kitPath;
            if (folderName.length() >= 5 && folderName.substr(folderName.length() - 5) == ".bbox") {
                folderName = folderName.substr(0, folderName.length() - 5);
            }
            system::createDirectory(folderName);
            
            std::string finalKitFile = folderName + "/kit.bbox";

            size_t lastSlash = folderName.find_last_of("/\\");
            kitName = (lastSlash == std::string::npos) ? folderName : folderName.substr(lastSlash + 1);

            json_t* root = kitToJson(folderName);
            json_dump_file(root, finalKitFile.c_str(), JSON_INDENT(4));
            json_decref(root);
            free(path);
        }
    }

    void loadKit() {
        std::string defaultDir = asset::plugin(pluginInstance, "kits");
        system::createDirectory(defaultDir);

        char* path = osdialog_file(OSDIALOG_OPEN, defaultDir.c_str(), NULL, osdialog_filters_parse("BBOX Kits:bbox"));
        if (path) {
            json_error_t error;
            json_t* root = json_load_file(path, 0, &error);
            if (root) {
                kitFromJson(root);
                json_decref(root);
            }
            free(path);
        }
    }
};

// --- Custom Functional Widgets ---

// 1. Fully custom clickable button with its own rendering
struct ActionButton : TransparentWidget {
    std::string text;
    std::function<void()> onClick;
    bool isPressed = false;

    void draw(const DrawArgs& args) override {
        nvgBeginPath(args.vg);
        nvgRoundedRect(args.vg, 0, 0, box.size.x, box.size.y, 2.0f);
        nvgFillColor(args.vg, isPressed ? nvgRGB(150, 150, 150) : nvgRGB(200, 200, 200));
        nvgFill(args.vg);
        nvgStrokeColor(args.vg, nvgRGB(100, 100, 100));
        nvgStroke(args.vg);

        if (!text.empty()) {
            if (APP->window->uiFont) nvgFontFaceId(args.vg, APP->window->uiFont->handle);
            nvgFillColor(args.vg, nvgRGB(30, 30, 30));
            nvgFontSize(args.vg, 12);
            nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
            nvgText(args.vg, box.size.x / 2.f, box.size.y / 2.f + 1, text.c_str(), NULL);
        }
    }

void onButton(const ButtonEvent& e) override {
        if (e.button == GLFW_MOUSE_BUTTON_LEFT) {
            if (e.action == GLFW_PRESS) {
                isPressed = true;
                e.consume(this);
            } else if (e.action == GLFW_RELEASE) {
                if (isPressed) {       // <-- Safety check
                    isPressed = false; // Reset before opening dialog
                    if (onClick) onClick();
                }
                e.consume(this);
            }
        }
    }
};

// 2. Button that retrieves its color dynamically (for Mute/Solo logic)
struct StateButton : TransparentWidget {
    std::string text;
    std::function<void()> onClick;
    std::function<NVGcolor()> getColor;
    bool isPressed = false;

    void draw(const DrawArgs& args) override {
        nvgBeginPath(args.vg);
        nvgRoundedRect(args.vg, 0, 0, box.size.x, box.size.y, 2.0f);
        nvgFillColor(args.vg, getColor ? getColor() : nvgRGB(150, 150, 150));
        nvgFill(args.vg);
        
        if (!text.empty()) {
            if (APP->window->uiFont) nvgFontFaceId(args.vg, APP->window->uiFont->handle);
            nvgFillColor(args.vg, nvgRGB(30, 30, 30));
            nvgFontSize(args.vg, 10);
            nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
            nvgText(args.vg, box.size.x / 2.f, box.size.y / 2.f + 1, text.c_str(), NULL);
        }
    }

    void onButton(const ButtonEvent& e) override {
        if (e.button == GLFW_MOUSE_BUTTON_LEFT) {
            if (e.action == GLFW_PRESS) {
		isPressed = true;
		e.consume(this);
	    }
            else if (e.action == GLFW_RELEASE) {
                if (isPressed) {       // <-- Safety check
                    isPressed = false; 
                    if (onClick) onClick();
                }
                e.consume(this);
            }
        }
    }
};

// 3. The Sample Viewer with Drag & Drop capability
struct SampleDisplayWidget : TransparentWidget {
    BBOX* module;
    int trackId;

    void draw(const DrawArgs& args) override {
        nvgBeginPath(args.vg);
        nvgRoundedRect(args.vg, 0, 0, box.size.x, box.size.y, 2.0f);
        nvgFillColor(args.vg, nvgRGB(230, 230, 230));
        nvgFill(args.vg);
        nvgStrokeColor(args.vg, nvgRGB(100, 100, 100));
        nvgStroke(args.vg);

        if (module) {
            if (APP->window->uiFont) nvgFontFaceId(args.vg, APP->window->uiFont->handle);
            nvgFillColor(args.vg, nvgRGB(30, 30, 30));
            nvgFontSize(args.vg, 12);
            nvgTextAlign(args.vg, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE);
            std::string text = module->samples[trackId].name.empty() ? "Drop WAV..." : module->samples[trackId].name;
            if(text.length() > 13) text = text.substr(0, 13) + "..";
            nvgText(args.vg, 5, box.size.y / 2.f + 1, text.c_str(), NULL);
        }
    }

    void onPathDrop(const PathDropEvent& e) override {
        if (!e.paths.empty() && module) {
            module->loadSample(trackId, e.paths[0]);
            e.consume(this);
        }
    }
};

// 4. Global Overlay Layer for rendering text reliably ON TOP of the SVG background
struct TextOverlayLayer : TransparentWidget {
    std::shared_ptr<window::Font> fontRegular;
    std::shared_ptr<window::Font> fontBold;

    void draw(const DrawArgs& args) override {
        // Fallback to UI font if custom font load failed
        if (!fontRegular) fontRegular = APP->window->uiFont;
        if (!fontBold) fontBold = APP->window->uiFont;
        if (!fontRegular || !fontBold) return;

        nvgFillColor(args.vg, nvgRGB(30, 30, 30));
        nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM);
        
        // Main Title
        nvgFontFaceId(args.vg, fontBold->handle);
        nvgFontSize(args.vg, 16);
        nvgText(args.vg, 375, 372, "BBOX", NULL);

        nvgFontFaceId(args.vg, fontRegular->handle);
        nvgFontSize(args.vg, 11);
        
        // Top Labels
        nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_TOP);
        nvgText(args.vg, 490, 10, "GMUTE", NULL);
        nvgText(args.vg, 550, 10, "GVOL", NULL);
        nvgText(args.vg, 650, 10, "MIX L", NULL);
        nvgText(args.vg, 690, 10, "MIX R", NULL);

        // Track Strip Column Headers
        float yH = 60;
        nvgText(args.vg, 154.5, yH, "M", NULL);
        nvgText(args.vg, 173.5, yH, "S", NULL);
        nvgText(args.vg, 198, yH, "GATE", NULL);
        nvgText(args.vg, 252.5, yH, "VEL", NULL);
        nvgText(args.vg, 332.5, yH, "TUNE", NULL);
        nvgText(args.vg, 412.5, yH, "PNCH", NULL);
        nvgText(args.vg, 492.5, yH, "DECY", NULL);
        nvgText(args.vg, 572.5, yH, "PAN", NULL);
        nvgText(args.vg, 652.5, yH, "VOL", NULL);
        nvgText(args.vg, 710, yH, "OUT", NULL);
    }
};

struct BBOXWidget : ModuleWidget {
    BBOXWidget(BBOX* module) {
        setModule(module);

	// Add screws
        setPanel(createPanel(asset::plugin(pluginInstance, "res/BBOX.svg")));
                addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        // Add Text Overlay First (so it draws over the panel, but beneath the knobs)
        TextOverlayLayer* overlay = new TextOverlayLayer();
        overlay->box.size = Vec(750, 380);
        overlay->fontRegular = APP->window->loadFont(asset::system("res/fonts/Roboto-Regular.ttf"));
        overlay->fontBold = APP->window->loadFont(asset::system("res/fonts/Roboto-Bold.ttf"));
        addChild(overlay);

        // Global Action Buttons
        ActionButton* kitBtn = new ActionButton();
        kitBtn->box.pos = Vec(20, 20);
        kitBtn->box.size = Vec(130, 25);
        kitBtn->text = module ? module->kitName : "Default Kit";
        kitBtn->onClick = [module, kitBtn]() { 
            if(module) { module->loadKit(); kitBtn->text = module->kitName; } 
        };
        addChild(kitBtn);

        ActionButton* saveBtn = new ActionButton();
        saveBtn->box.pos = Vec(160, 20);
        saveBtn->box.size = Vec(50, 25);
        saveBtn->text = "SAVE";
        saveBtn->onClick = [module]() { if(module) module->saveKit(); };
        addChild(saveBtn);

        ActionButton* mgroupBtn = new ActionButton();
        mgroupBtn->box.pos = Vec(220, 20);
        mgroupBtn->box.size = Vec(80, 25);
        mgroupBtn->text = "MUTE GRPS";
        mgroupBtn->onClick = [module, this]() {
            if(!module) return;
            ui::Menu* menu = createMenu();
            menu->addChild(createMenuLabel("MUTE GROUPS"));
            for (int i = 0; i < 8; i++) {
                menu->addChild(createSubmenuItem(string::f("Track %d", i+1), "",[=](ui::Menu* m) {
                    for(int g = 0; g < 4; g++) {
                        m->addChild(createCheckMenuItem(string::f("Group %d", g+1), "",
                            [=]() { return (module->muteGroups[i] & (1<<g)) != 0; },
                            [=]() { module->muteGroups[i] ^= (1<<g); }
                        ));
                    }
                }));
            }
        };
        addChild(mgroupBtn);
	ActionButton* clearAllBtn = new ActionButton();
        clearAllBtn->box.pos = Vec(310, 20);
        clearAllBtn->box.size = Vec(70, 25);
        clearAllBtn->text = "CLEAR ALL";
        clearAllBtn->onClick = [module]() {
            if(module) module->clearAllSamples();
        };
        addChild(clearAllBtn);

        // Global Ports
        addParam(createParamCentered<LEDButton>(Vec(490, 36), module, BBOX::GLOBAL_MUTE_PARAM));
        addParam(createParamCentered<RoundBlackKnob>(Vec(550, 36), module, BBOX::GLOBAL_VOL_PARAM));
        addOutput(createOutputCentered<PJ301MPort>(Vec(650, 36), module, BBOX::MIX_L_OUTPUT));
        addOutput(createOutputCentered<PJ301MPort>(Vec(690, 36), module, BBOX::MIX_R_OUTPUT));

        // 8 Track Strips
        for (int i = 0; i < 8; i++) {
            float y = 85 + i * 36;
            
            SampleDisplayWidget* sdw = new SampleDisplayWidget();
            sdw->box.pos = Vec(10, y - 10);
            sdw->box.size = Vec(100, 20);
            sdw->module = module;
            sdw->trackId = i;
            addChild(sdw);

            ActionButton* loadBtn = new ActionButton();
            loadBtn->box.pos = Vec(105, y - 10);
            loadBtn->box.size = Vec(20, 20);
            loadBtn->text = "...";
            loadBtn->onClick = [module, i]() {
                if(!module) return;
                char* path = osdialog_file(OSDIALOG_OPEN, NULL, NULL, osdialog_filters_parse("Audio:wav"));
                if(path) { module->loadSample(i, path); free(path); }
            };
            addChild(loadBtn);

            ActionButton* clearBtn = new ActionButton();
            clearBtn->box.pos = Vec(128, y - 10);
            clearBtn->box.size = Vec(15, 20);
            clearBtn->text = "X";
            clearBtn->onClick =[module, i]() {
                if(module) module->clearSample(i);
            };
            addChild(clearBtn);

            StateButton* muteBtn = new StateButton();
            muteBtn->box.pos = Vec(147, y - 8);
            muteBtn->box.size = Vec(15, 15);
            muteBtn->onClick = [module, i]() {
                if(!module) return;
                bool anySolo = false;
                for(int j=0; j<8; j++) if(module->isSolo[j]) anySolo = true;
                if(anySolo && !module->isSolo[i]) module->forceUnmute[i] = !module->forceUnmute[i];
                else module->isMuted[i] = !module->isMuted[i];
            };
            muteBtn->getColor = [module, i]() -> NVGcolor {
                if (!module) return nvgRGB(150, 150, 150);
                if (module->isMuted[i]) return nvgRGB(255, 80, 80);
                if (module->forceUnmute[i]) return nvgRGB(80, 255, 80);
                return nvgRGB(150, 150, 150);
            };
            addChild(muteBtn);

            StateButton* soloBtn = new StateButton();
            soloBtn->box.pos = Vec(167, y - 8);
            soloBtn->box.size = Vec(15, 15);
            soloBtn->onClick = [module, i]() {
                if(module) module->isSolo[i] = !module->isSolo[i];
            };
            soloBtn->getColor = [module, i]() -> NVGcolor {
                return (module && module->isSolo[i]) ? nvgRGB(255, 255, 80) : nvgRGB(150, 150, 150);
            };
            addChild(soloBtn);

            addInput(createInputCentered<PJ301MPort>(Vec(198, y), module, BBOX::GATE_INPUT + i));
            addInput(createInputCentered<PJ301MPort>(Vec(235, y), module, BBOX::VEL_INPUT + i));
            addParam(createParamCentered<RoundBlackKnob>(Vec(270, y), module, BBOX::VEL_PARAM + i));

            addInput(createInputCentered<PJ301MPort>(Vec(315, y), module, BBOX::TUNE_INPUT + i));
            addParam(createParamCentered<RoundBlackKnob>(Vec(350, y), module, BBOX::TUNE_PARAM + i));

            addInput(createInputCentered<PJ301MPort>(Vec(395, y), module, BBOX::PUNCH_INPUT + i));
            addParam(createParamCentered<RoundBlackKnob>(Vec(430, y), module, BBOX::PUNCH_PARAM + i));

            addInput(createInputCentered<PJ301MPort>(Vec(475, y), module, BBOX::DECAY_INPUT + i));
            addParam(createParamCentered<RoundBlackKnob>(Vec(510, y), module, BBOX::DECAY_PARAM + i));

            addInput(createInputCentered<PJ301MPort>(Vec(555, y), module, BBOX::PAN_INPUT + i));
            addParam(createParamCentered<RoundBlackKnob>(Vec(590, y), module, BBOX::PAN_PARAM + i));

            addInput(createInputCentered<PJ301MPort>(Vec(635, y), module, BBOX::VOL_INPUT + i));
            addParam(createParamCentered<RoundBlackKnob>(Vec(670, y), module, BBOX::VOL_PARAM + i));

            addOutput(createOutputCentered<PJ301MPort>(Vec(710, y), module, BBOX::TRK_OUTPUT + i));
        }
    }
};

Model* modelBBOX = createModel<BBOX, BBOXWidget>("BBOX");