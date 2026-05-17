#include "plugin.hpp"
#include <cmath>
#include <string>
#include <algorithm>

// Include standard LAME library (Must link with libmp3lame.a in Makefile)
#include <lame/lame.h>

// Include minimp3 (Must place minimp3.h in your src/ directory)
#define MINIMP3_IMPLEMENTATION
#define MINIMP3_FLOAT_OUTPUT
#include "minimp3.h"

using namespace rack;

const int MP3_RATES[] = {8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000};
const int NUM_MP3_RATES = 9;
const int MP3_BITRATES[] = {8, 16, 24, 32, 40, 48, 56, 64, 80, 96, 112, 128, 144, 160, 192, 224, 256, 320};

// Helper function to calculate safe MP3 parameters
void getSafeMp3Params(float rateParam, float srParam, int& out_sr, int& final_br) {
    int sr_idx = std::round(srParam * (NUM_MP3_RATES - 1));
    out_sr = MP3_RATES[sr_idx];
    
    int min_br = 16, max_br = 320;
    if (out_sr < 16000) { min_br = 8; max_br = 160; }
    else if (out_sr < 32000) { min_br = 8; max_br = 160; }

    int target_br = min_br + rateParam * (max_br - min_br);
    final_br = target_br;
    int best_diff = 999;
    for(int b : MP3_BITRATES) {
        if (b >= min_br && b <= max_br) {
            if (std::abs(b - target_br) < best_diff) {
                best_diff = std::abs(b - target_br);
                final_br = b;
            }
        }
    }
}

struct BYT : Module {
    enum ParamIds { MODE_SWITCH, RATE_KNOB, FILTER_KNOB, SR_KNOB, NUM_PARAMS };
    enum InputIds { RATE_CV, FILTER_CV, SR_CV, IN_L, IN_R, NUM_INPUTS };
    enum OutputIds { OUT_L, OUT_R, NUM_OUTPUTS };

    float heldL = 0.f, heldR = 0.f;
    float nextL = 0.f, nextR = 0.f;
    
    // --- WAV STATE ---
    float phase = 0.f;
    float err[2][3] = {{0.f}};
    
    // --- MP3 STATE ---
    lame_global_flags* lame;
    mp3dec_t mp3d;
    
    dsp::RingBuffer<float, 8192> inBufL;
    dsp::RingBuffer<float, 8192> inBufR;
    
    dsp::RingBuffer<float, 32768> outBufL;
    dsp::RingBuffer<float, 32768> outBufR;

    uint8_t mp3_stream[32768];
    int mp3_stream_bytes = 0;

    int last_bitrate = -1;
    int last_sr = -1;
    int last_filter = -1;
    int mp3_reinit_cooldown = 0;

    float mp3_read_posL = 0.f;
    int current_mp3_sr = 48000;

    BYT() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS);
        
        configParam(MODE_SWITCH, 0.f, 1.f, 0.f, "Mode (0=MP3, 1=WAV)");
        configParam(RATE_KNOB, 0.f, 1.f, 0.5f, "Rate/Bit Depth");
        configParam(FILTER_KNOB, 0.f, 1.f, 0.5f, "Filter/Dither Mode");
        configParam(SR_KNOB, 0.f, 1.f, 1.f, "Sample Rate");
        
        configBypass(IN_L, OUT_L);
        configBypass(IN_R, OUT_R);

        memset(mp3_stream, 0, sizeof(mp3_stream));

        lame = lame_init();
        lame_set_num_channels(lame, 2);
        lame_set_in_samplerate(lame, 48000);
        lame_init_params(lame);

        mp3dec_init(&mp3d);
    }

    ~BYT() {
        if (lame) lame_close(lame);
    }

    void process(const ProcessArgs& args) override {
        bool isWav = params[MODE_SWITCH].getValue() > 0.5f;

        float rateParam = clamp(params[RATE_KNOB].getValue() + inputs[RATE_CV].getVoltage() * 0.1f, 0.f, 1.f);
        float filterParam = clamp(params[FILTER_KNOB].getValue() + inputs[FILTER_CV].getVoltage() * 0.1f, 0.f, 1.f);
        float srParam = clamp(params[SR_KNOB].getValue() + inputs[SR_CV].getVoltage() * 0.1f, 0.f, 1.f);

        float inL = inputs[IN_L].isConnected() ? inputs[IN_L].getVoltage() : 0.f;
        float inR = inputs[IN_R].isConnected() ? inputs[IN_R].getVoltage() : inL;

        if (isWav) {
            // ==================== WAV MODE (Untouched) ====================
            float target_sr = srParam * 48000.f;
            if (target_sr < 0.1f) target_sr = 0.1f;

            bool update_sample = false;
            if (target_sr >= args.sampleRate * 0.99f) {
                update_sample = true;
                phase = 0.f;
            } else {
                phase += target_sr * args.sampleTime;
                if (phase >= 1.f) {
                    phase -= 1.f;
                    update_sample = true;
                }
            }

            if (update_sample) {
                float bitDepth = 1.f + rateParam * 31.f; 
                int ditherMode = std::round(filterParam * 5.f);

                float levels = powf(2.f, bitDepth);
                float q = 10.f / levels; 
                float in[2] = {inL, inR};

                for(int c = 0; c < 2; c++) {
                    float x = in[c];
                    float dither = 0.f;
                    float error_feedback = 0.f;

                    switch(ditherMode) {
                        case 0: break; 
                        case 1: dither = (random::uniform() - 0.5f) * q; break; 
                        case 2: dither = (random::uniform() - random::uniform()) * q; break; 
                        case 3: dither = (random::uniform() - random::uniform()) * q; break;
                        case 4: dither = (random::uniform() - random::uniform()) * q; error_feedback = err[c][0] * 0.5f; break;
                        case 5: dither = (random::uniform() - random::uniform()) * q; error_feedback = err[c][0] * 1.0f - err[c][1] * 0.5f; break;
                    }

                    float x_dithered = x + dither + error_feedback;
                    float x_quantized = std::round(x_dithered / q) * q;

                    err[c][2] = err[c][1];
                    err[c][1] = err[c][0];
                    err[c][0] = x_quantized - x;

                    if (c == 0) heldL = x_quantized;
                    else heldR = x_quantized;
                }
            }
            outputs[OUT_L].setVoltage(heldL);
            outputs[OUT_R].setVoltage(heldR);

        } else {
            // ==================== MP3 MODE ====================
            if (!inBufL.full() && !inBufR.full()) {
                inBufL.push(inL / 5.f); 
                inBufR.push(inR / 5.f);
            }

            if (mp3_reinit_cooldown > 0) mp3_reinit_cooldown--;

            const int BLOCK_SIZE = 1152;
            
            if (inBufL.size() >= BLOCK_SIZE) {
                int out_sr, final_br;
                getSafeMp3Params(rateParam, srParam, out_sr, final_br);
                
                // Quantize to 20 steps to prevent CV noise from infinitely crashing LAME
                int filter_val = std::round(filterParam * 20.f);

                if (out_sr != last_sr || final_br != last_bitrate || filter_val != last_filter) {
                    if (mp3_reinit_cooldown == 0) {
                        last_sr = out_sr;
                        last_bitrate = final_br;
                        last_filter = filter_val;

                        // CRITICAL FIX: You MUST completely recreate the LAME instance to avoid state corruption!
                        if (lame) lame_close(lame);
                        lame = lame_init();

                        lame_set_num_channels(lame, 2);
                        lame_set_in_samplerate(lame, args.sampleRate);
                        lame_set_out_samplerate(lame, out_sr);
                        lame_set_brate(lame, final_br);
                        lame_set_bWriteVbrTag(lame, 0); 
                        lame_set_disable_reservoir(lame, 1);
                        
                        if (filterParam > 0.05f) {
                            int lp = 4000 + filterParam * 20000; 
                            
                            // Prevent LAME mathematically failing if filter exceeds Nyquist
                            int max_lp = (out_sr / 2) - 100;
                            if (max_lp < 500) max_lp = 500;
                            if (lp > max_lp) lp = max_lp;
                            
                            lame_set_lowpassfreq(lame, lp); 
                            lame_set_quality(lame, 7); // Safe & Fast Mode
                        } else {
                            lame_set_lowpassfreq(lame, 0);  
                            lame_set_quality(lame, 5);      
                        }
                        
                        if (lame_init_params(lame) < 0) {
                            // Safe fallback in case LAME engine rejects the wild parameters
                            lame_set_out_samplerate(lame, 44100);
                            lame_set_brate(lame, 128);
                            lame_set_lowpassfreq(lame, 0);
                            lame_init_params(lame);
                        }
                        
                        outBufL.clear();
                        outBufR.clear();
                        mp3_stream_bytes = 0;
                        mp3dec_init(&mp3d);
                        
                        mp3_reinit_cooldown = args.sampleRate * 0.25f; // 250ms cooldown
                    }
                }

                float blockL[BLOCK_SIZE], blockR[BLOCK_SIZE];
                for(int i=0; i<BLOCK_SIZE; i++) {
                    blockL[i] = inBufL.shift();
                    blockR[i] = inBufR.shift();
                }

                // CRITICAL FIX: LAME mathematically panics & returns -1 if size < 8640.
                unsigned char lame_out[16384]; 
                int bytes = lame_encode_buffer_ieee_float(lame, blockL, blockR, BLOCK_SIZE, lame_out, sizeof(lame_out));

                if (bytes > 0) {
                    if (mp3_stream_bytes + bytes < 32768) {
                        memcpy(mp3_stream + mp3_stream_bytes, lame_out, bytes);
                        mp3_stream_bytes += bytes;
                    } else {
                        mp3_stream_bytes = 0;
                        memcpy(mp3_stream, lame_out, bytes);
                        mp3_stream_bytes = bytes;
                        mp3dec_init(&mp3d);
                    }
                }

                int offset = 0;
                while (offset < mp3_stream_bytes) {
                    mp3dec_frame_info_t info;
                    float pcm[2304]; 
                    int decoded_samples = mp3dec_decode_frame(&mp3d, mp3_stream + offset, mp3_stream_bytes - offset, pcm, &info);
                    
                    if (info.frame_bytes == 0) break; 

                    if (decoded_samples > 0 && info.channels > 0) {
                        if (info.hz > 0) current_mp3_sr = info.hz;
                        for (int i = 0; i < decoded_samples; i++) {
                            if (!outBufL.full()) outBufL.push(pcm[i * info.channels] * 5.f);
                            if (!outBufR.full()) {
                                float right_val = (info.channels == 2) ? pcm[i * info.channels + 1] : pcm[i * info.channels];
                                outBufR.push(right_val * 5.f);
                            }
                        }
                    }
                    offset += info.frame_bytes;
                }

                if (offset > 0) {
                    if (offset < mp3_stream_bytes) {
                        memmove(mp3_stream, mp3_stream + offset, mp3_stream_bytes - offset);
                        mp3_stream_bytes -= offset;
                    } else {
                        mp3_stream_bytes = 0;
                    }
                }
            }

            // Engine-to-MP3 Resampling and Playback
            if (outBufL.size() == 0) {
                outputs[OUT_L].setVoltage(heldL);
                outputs[OUT_R].setVoltage(heldR);
                return; 
            }

            float playback_speed = (float)current_mp3_sr / args.sampleRate;
            mp3_read_posL += playback_speed;
            
            if (mp3_read_posL >= 1.f) {
                int jumps = (int)mp3_read_posL;
                mp3_read_posL -= jumps;
                for (int i = 0; i < jumps; i++) {
                    heldL = nextL;
                    heldR = nextR;
                    if (outBufL.size() > 0) nextL = outBufL.shift();
                    if (outBufR.size() > 0) nextR = outBufR.shift();
                }
            }
            
            float outL = heldL + (nextL - heldL) * mp3_read_posL;
            float outR = heldR + (nextR - heldR) * mp3_read_posL;

            outputs[OUT_L].setVoltage(outL);
            outputs[OUT_R].setVoltage(outR);
        }
    }
};

// ==============================================================================
// CUSTOM SYSTEM FONT TEXT WIDGET
// ==============================================================================
struct SystextWidget : Widget {
    BYT* module = nullptr;
    int type = -1;
    std::string text = "";
    float fontSize = 12.f;

    void draw(const DrawArgs& args) override {
        std::shared_ptr<Font> font = APP->window->uiFont;
        if (!font) return;

        std::string displayText = text;

        if (module && type >= 0) {
            bool isWav = module->params[BYT::MODE_SWITCH].getValue() > 0.5f;
            
            float rawRate = clamp(module->params[BYT::RATE_KNOB].getValue() + module->inputs[BYT::RATE_CV].getVoltage() * 0.1f, 0.f, 1.f);
            float rawFilter = clamp(module->params[BYT::FILTER_KNOB].getValue() + module->inputs[BYT::FILTER_CV].getVoltage() * 0.1f, 0.f, 1.f);
            float rawSR = clamp(module->params[BYT::SR_KNOB].getValue() + module->inputs[BYT::SR_CV].getVoltage() * 0.1f, 0.f, 1.f);

            if (type == 0) { 
                displayText = isWav ? "BIT DEPTH" : "BITRATE";
            } else if (type == 1) { 
                displayText = isWav ? "DITHERING" : "FILTER";
            } else if (type == 2) { 
                if (isWav) {
                    float val = 1.f + rawRate * 31.f;
                    displayText = string::f("%.1f bit", val);
                } else {
                    int out_sr, final_br;
                    getSafeMp3Params(rawRate, rawSR, out_sr, final_br);
                    displayText = string::f("%d kbps", final_br);
                }
            } else if (type == 3) { 
                if (isWav) {
                    int mode = std::round(rawFilter * 5.f);
                    const char* modes[] = {"Off", "Rect", "Tri", "POW-r 1", "POW-r 2", "POW-r 3"};
                    displayText = modes[mode];
                } else {
                    float val = rawFilter * 100.f;
		    displayText = string::f("%.0f %%", val);
                }
            } else if (type == 4) { 
                if (isWav) {
                    float val = std::fmax(0.1f, rawSR * APP->engine->getSampleRate());
                    displayText = string::f("%.0f Hz", val);
                } else {
                    int out_sr, final_br;
                    getSafeMp3Params(rawRate, rawSR, out_sr, final_br);
                    displayText = string::f("%d Hz", out_sr);
                }
            }
        }

        nvgFontSize(args.vg, fontSize);
        nvgFontFaceId(args.vg, font->handle);
        nvgFillColor(args.vg, nvgRGB(50, 50, 50));
        nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
        nvgText(args.vg, box.size.x / 2.f, box.size.y / 2.f, displayText.c_str(), NULL);
    }
};

SystextWidget* addLabel(Vec center, std::string text, BYT* mod = nullptr, int type = -1, float fontSize = 11.f) {
    SystextWidget* w = new SystextWidget();
    w->box.size = Vec(100, 20);
    w->box.pos = center - w->box.size.div(2.f);
    w->text = text;
    w->module = mod;
    w->type = type;
    w->fontSize = fontSize;
    return w;
}

// ==============================================================================
// MODULE WIDGET (UI)
// ==============================================================================
struct BYTWidget : ModuleWidget {
    BYTWidget(BYT* module) {
        setModule(module);
        setPanel(APP->window->loadSvg(asset::plugin(pluginInstance, "res/byt.svg")));

        float L_COL = 45.f;
        float R_COL = 105.f;
        float M_COL = 75.f;

        addChild(addLabel(Vec(M_COL, 362), "BYT", nullptr, -1, 16.f));

        addChild(addLabel(Vec(M_COL, 59), "MP3", nullptr, -1, 10.f));
        addParam(createParamCentered<CKSS>(Vec(M_COL, 42), module, BYT::MODE_SWITCH));
        addChild(addLabel(Vec(M_COL, 25), "WAV", nullptr, -1, 10.f));

        addChild(addLabel(Vec(M_COL, 75), "", module, 0, 11.f));
        addChild(addLabel(Vec(R_COL, 90), "CV", nullptr, -1, 9.f));
        addParam(createParamCentered<RoundBlackKnob>(Vec(L_COL, 100), module, BYT::RATE_KNOB));
        addInput(createInputCentered<PJ301MPort>(Vec(R_COL, 110), module, BYT::RATE_CV));
        addChild(addLabel(Vec(L_COL, 120), "", module, 2, 10.f));

        addChild(addLabel(Vec(M_COL, 145), "", module, 1, 11.f));
        addChild(addLabel(Vec(R_COL, 160), "CV", nullptr, -1, 9.f));
        addParam(createParamCentered<RoundBlackKnob>(Vec(L_COL, 170), module, BYT::FILTER_KNOB));
        addInput(createInputCentered<PJ301MPort>(Vec(R_COL, 180), module, BYT::FILTER_CV));
        addChild(addLabel(Vec(L_COL, 190), "", module, 3, 10.f));

        addChild(addLabel(Vec(M_COL, 215), "SAMPLE RATE", nullptr, -1, 11.f));
        addChild(addLabel(Vec(R_COL, 230), "CV", nullptr, -1, 9.f));
        addParam(createParamCentered<RoundBlackKnob>(Vec(L_COL, 240), module, BYT::SR_KNOB));
        addInput(createInputCentered<PJ301MPort>(Vec(R_COL, 250), module, BYT::SR_CV));
        addChild(addLabel(Vec(L_COL, 260), "", module, 4, 10.f));

        addChild(addLabel(Vec(L_COL, 290), "AUDIO IN", nullptr, -1, 10.f));
        addChild(addLabel(Vec(R_COL, 290), "AUDIO OUT", nullptr, -1, 10.f));
        
        addChild(addLabel(Vec(30, 310), "L", nullptr, -1, 10.f));
        addChild(addLabel(Vec(60, 310), "R", nullptr, -1, 10.f));
        addChild(addLabel(Vec(90, 310), "L", nullptr, -1, 10.f));
        addChild(addLabel(Vec(120, 310), "R", nullptr, -1, 10.f));

        addInput(createInputCentered<PJ301MPort>(Vec(30, 330), module, BYT::IN_L));
        addInput(createInputCentered<PJ301MPort>(Vec(60, 330), module, BYT::IN_R));
        
	// Rack2 Screws
        addOutput(createOutputCentered<PJ301MPort>(Vec(90, 330), module, BYT::OUT_L));
        addOutput(createOutputCentered<PJ301MPort>(Vec(120, 330), module, BYT::OUT_R));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
    }
};

Model* modelBYT = createModel<BYT, BYTWidget>("BYT");