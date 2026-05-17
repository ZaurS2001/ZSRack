#include <rack.hpp>
#include <sstream>
#include <vector>
#include <string>
#include <mutex>

using namespace rack;

extern Plugin* pluginInstance;

struct Lingo : Module {
    enum ParamId {
        RUN_PARAM,
        RESET_PARAM,
        WEIGHT_PARAM,
        UPPER_VOLT_PARAM,
        LOWER_VOLT_PARAM,
        PARAMS_LEN
    };
    enum InputId {
        CLOCK_INPUT,
        RUN_INPUT,
        RESET_INPUT,
        WEIGHT_INPUT,
        UPPER_VOLT_INPUT,
        LOWER_VOLT_INPUT,
        INPUTS_LEN
    };
    enum OutputId {
        GATE_OUTPUT,
        CV_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId {
        RUN_LIGHT,
        LIGHTS_LEN
    };

    bool running = true;
    std::string text = "KICK\nHAT HAT HAT\nSNARE WAIT";
    bool textReady = false;

    // Polyphonic Mini-notation parsing
    std::vector<std::vector<std::string>> linesOfWords;
    std::vector<int> currentWordIndices;
    
    // Thread safety for updating text from the UI thread to the audio thread
    std::mutex dataMutex;

    // Display tracking for the UI (Changed to char to prevent allocation in process thread)
    char displayChar = '\0';
    int activeChannels = 1;

    // Timing and Ratcheting
    float timer = 0.0f;
    float clockInterval = 0.5f; // Defaults to 120 BPM
    float phase = 0.0f;

    dsp::SchmittTrigger clockTrigger;
    dsp::SchmittTrigger runTrigger;
    dsp::SchmittTrigger resetTrigger;

    Lingo() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        
        configParam(RUN_PARAM, 0.0, 1.0, 0.0, "Run Toggle");
        configParam(RESET_PARAM, 0.0, 1.0, 0.0, "Reset Sequence");
        configParam(WEIGHT_PARAM, -1.0, 1.0, 0.1, "Weight Depth", "%", 0.0f, 100.0f);
        configParam(UPPER_VOLT_PARAM, -10.0, 10.0, 8.0, "Uppercase Max CV Voltage", "V");
        configParam(LOWER_VOLT_PARAM, -10.0, 10.0, 5.0, "Lowercase Max CV Voltage", "V");

        configInput(CLOCK_INPUT, "External Clock (1 Beat = 1 Word)");
        configInput(RUN_INPUT, "Run Toggle CV");
        configInput(RESET_INPUT, "Reset CV");
        configInput(WEIGHT_INPUT, "Weight Depth CV (Polyphonic)");
        configInput(UPPER_VOLT_INPUT, "Upper Max Volt CV (Polyphonic)");
        configInput(LOWER_VOLT_INPUT, "Lower Max Volt CV (Polyphonic)");

        configOutput(GATE_OUTPUT, "Polyphonic Gate (Up to 16 Channels)");
        configOutput(CV_OUTPUT, "Polyphonic Text CV (Up to 16 Channels)");

        // Initialize default text
        setAndParseText(text);
    }

    // This is called from the UI thread (or on load) to prevent audio dropouts
    void setAndParseText(const std::string& newText) {
        std::vector<std::vector<std::string>> newLinesOfWords;
        std::istringstream textStream(newText);
        std::string line;
        
        // Parse each line 
        while (std::getline(textStream, line, '\n')) {
            std::vector<std::string> wordsInLine;
            std::istringstream lineStream(line);
            std::string word;
            while (lineStream >> word) {
                wordsInLine.push_back(word);
            }
            if (wordsInLine.empty()) {
                wordsInLine.push_back(""); // Empty line yields silence
            }
            newLinesOfWords.push_back(wordsInLine);
        }
        
        if (newLinesOfWords.empty()) {
            newLinesOfWords.push_back({""});
        }

        // Safely swap the new data into the audio thread using a mutex
        std::lock_guard<std::mutex> lock(dataMutex);
        
        text = newText;
        linesOfWords = newLinesOfWords;
        
        if (currentWordIndices.size() < linesOfWords.size()) {
            currentWordIndices.resize(linesOfWords.size(), 0);
        }
        
        activeChannels = std::min((int)linesOfWords.size(), 16);
    }

    void process(const ProcessArgs& args) override {
        if (runTrigger.process(inputs[RUN_INPUT].getVoltage() + params[RUN_PARAM].getValue())) {
            running = !running;
        }
        
        if (resetTrigger.process(inputs[RESET_INPUT].getVoltage() + params[RESET_PARAM].getValue())) {
            std::lock_guard<std::mutex> lock(dataMutex);
            std::fill(currentWordIndices.begin(), currentWordIndices.end(), 0);
            phase = 0.0f;
            timer = 0.0f;
        }

        timer += args.sampleTime;

        // Lock data arrays so they don't change while we read them
        std::lock_guard<std::mutex> lock(dataMutex);

        // Clock processing
        if (clockTrigger.process(inputs[CLOCK_INPUT].getVoltage())) {
            if (timer > 0.01f) clockInterval = timer;
            timer = 0.0f;
            phase = 0.0f;

            if (running) {
                // Advance every active line to its next word independently
                for (size_t i = 0; i < linesOfWords.size(); ++i) {
                    if (!linesOfWords[i].empty() && linesOfWords[i][0] != "") {
                        currentWordIndices[i] = (currentWordIndices[i] + 1) % linesOfWords[i].size();
                    }
                }
            }
        }

        // Set dynamic polyphony channel counts
        outputs[CV_OUTPUT].setChannels(activeChannels);
        outputs[GATE_OUTPUT].setChannels(activeChannels);

        // ALWAYS loop through all 16 channels to prevent "Ghost" Voltages
        for (int ch = 0; ch < 16; ch++) {
            // If the channel is beyond our active lines, or empty, FORCE it to 0V
            if (ch >= activeChannels || !running || linesOfWords[ch].empty() || linesOfWords[ch][0] == "") {
                outputs[CV_OUTPUT].setVoltage(0.0f, ch);
                outputs[GATE_OUTPUT].setVoltage(0.0f, ch);
                if (ch == 0) displayChar = '\0';
                continue;
            }

            phase = timer / clockInterval;
            if (phase > 1.0f) phase = 1.0f; 

            std::string currentWord = linesOfWords[ch][currentWordIndices[ch] % linesOfWords[ch].size()];
            int wordLen = currentWord.length();
            
            int charIndex = (int)(phase * wordLen);
            if (charIndex >= wordLen) charIndex = wordLen - 1;
            
            char c = currentWord[charIndex];
            
            // UI Display uses Channel 1's character (saved as primitive char to avoid allocation)
            if (ch == 0) displayChar = c;
            
            float charPhase = (phase * wordLen) - charIndex;

            // --- POLYPHONIC CV LOGIC ---
            float targetCV = 0.0f;
            if (c >= 'a' && c <= 'z') {
                float maxLower = params[LOWER_VOLT_PARAM].getValue() + inputs[LOWER_VOLT_INPUT].getPolyVoltage(ch);
                targetCV = ((c - 'a' + 1) / 26.0f) * maxLower;
            } else if (c >= 'A' && c <= 'Z') {
                float maxUpper = params[UPPER_VOLT_PARAM].getValue() + inputs[UPPER_VOLT_INPUT].getPolyVoltage(ch);
                targetCV = ((c - 'A' + 1) / 26.0f) * maxUpper;
            } else if (c >= '0' && c <= '9') {
                targetCV = ((c - '0' + 1) / 10.0f) * 10.0f;
            }
            
            float weight = params[WEIGHT_PARAM].getValue() + (inputs[WEIGHT_INPUT].getPolyVoltage(ch) / 5.0f);
            weight = clamp(weight, -1.0f, 1.0f);
            outputs[CV_OUTPUT].setVoltage(targetCV * weight, ch);

            // --- POLYPHONIC VOWEL GATE LOGIC ---
            bool isVowel = (c == 'a' || c == 'e' || c == 'i' || c == 'o' || c == 'u' || c == 'y' ||
                            c == 'A' || c == 'E' || c == 'I' || c == 'O' || c == 'U' || c == 'Y');
            
            if (isVowel && charPhase < 0.5f && phase < 1.0f) {
                outputs[GATE_OUTPUT].setVoltage(5.0f, ch);
            } else {
                outputs[GATE_OUTPUT].setVoltage(0.0f, ch); 
            }
        }

        lights[RUN_LIGHT].setBrightness(running ? 1.0f : 0.0f);
    }

    json_t* dataToJson() override {
        json_t* rootJ = json_object();
        std::lock_guard<std::mutex> lock(dataMutex);
        json_object_set_new(rootJ, "text", json_string(text.c_str()));
        json_object_set_new(rootJ, "running", json_boolean(running));
        return rootJ;
    }

    void dataFromJson(json_t* rootJ) override {
        json_t* textJ = json_object_get(rootJ, "text");
        if (textJ) {
            setAndParseText(json_string_value(textJ));
        }

        json_t* runningJ = json_object_get(rootJ, "running");
        if (runningJ) running = json_boolean_value(runningJ);
        
        textReady = false; 
    }
};

// --- Custom UI Widgets ---

struct BlackLabel : Widget {
    std::string text;
    int fontSize;

    BlackLabel(Vec pos, std::string text, int fontSize = 12) {
        this->box.pos = pos;
        this->text = text;
        this->fontSize = fontSize;
    }

    void draw(const DrawArgs& args) override {
        nvgFillColor(args.vg, nvgRGB(0, 0, 0));
        nvgFontSize(args.vg, fontSize);
        if (APP->window->uiFont) {
            nvgFontFaceId(args.vg, APP->window->uiFont->handle);
        }
        nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_TOP);
        nvgText(args.vg, 0, 0, text.c_str(), NULL);
    }
};

struct CharDisplay : Widget {
    Lingo* module;
    CharDisplay(Vec pos, Lingo* mod) {
        box.pos = pos;
        box.size = Vec(40, 40);
        module = mod;
    }

    void draw(const DrawArgs& args) override {
        nvgBeginPath(args.vg);
        nvgRoundedRect(args.vg, 0, 0, box.size.x, box.size.y, 4.0f);
        nvgFillColor(args.vg, nvgRGB(10, 10, 10));
        nvgFill(args.vg);
        
        if (module) {
            // Draw Main Character (Channel 1)
            char c = module->displayChar;
            if (c != '\0') {
                std::string s(1, c);
                
                bool isUpperVowel = (c == 'A' || c == 'E' || c == 'I' || c == 'O' || c == 'U' || c == 'Y');
                bool isLowerVowel = (c == 'a' || c == 'e' || c == 'i' || c == 'o' || c == 'u' || c == 'y');
                bool isUpperConsonant = (c >= 'A' && c <= 'Z' && !isUpperVowel);
                bool isLowerConsonant = (c >= 'a' && c <= 'z' && !isLowerVowel);

                if (isUpperVowel) nvgFillColor(args.vg, nvgRGB(255, 50, 50));        
                else if (isLowerVowel) nvgFillColor(args.vg, nvgRGB(255, 150, 150)); 
                else if (isUpperConsonant) nvgFillColor(args.vg, nvgRGB(50, 255, 50)); 
                else if (isLowerConsonant) nvgFillColor(args.vg, nvgRGB(150, 255, 150)); 
                else nvgFillColor(args.vg, nvgRGB(150, 150, 150)); 

                nvgFontSize(args.vg, 26);
                if (APP->window->uiFont) nvgFontFaceId(args.vg, APP->window->uiFont->handle);
                nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
                nvgText(args.vg, box.size.x / 2, box.size.y / 2, s.c_str(), NULL);
            }

            // Draw Polyphony Count in top right corner
            nvgFillColor(args.vg, nvgRGB(0, 200, 255));
            nvgFontSize(args.vg, 10);
            nvgTextAlign(args.vg, NVG_ALIGN_RIGHT | NVG_ALIGN_TOP);
            std::string voiceText = std::to_string(module->activeChannels) + "V";
            nvgText(args.vg, box.size.x - 3, 3, voiceText.c_str(), NULL);
        }
    }
};

struct LingoTextField : ui::TextField {
    Lingo* module;
    
    LingoTextField() {
        multiline = true;
    }
    
    void step() override {
        ui::TextField::step();
        
        if (module && !module->textReady) {
            text = module->text;
            module->textReady = true;
        }

        int charsPerLine = 38;
        float lineHeight = 16.0f;
        int totalLines = 0;
        int currentLineLen = 0;

        for (char c : text) {
            if (c == '\n') {
                totalLines++;
                currentLineLen = 0;
            } else {
                currentLineLen++;
                if (currentLineLen >= charsPerLine) {
                    totalLines++;
                    currentLineLen = 0;
                }
            }
        }
        totalLines++; 

        float dynamicHeight = (totalLines * lineHeight) + 30.0f;
        box.size.y = std::max(280.0f, dynamicHeight);
    }
    
    void onChange(const event::Change& e) override {
        ui::TextField::onChange(e);
        // Call the thread-safe parser function instead of writing string directly
        if (module) module->setAndParseText(text);
    }
};

struct LingoWidget : ModuleWidget {
    LingoWidget(Lingo* module) {
        setModule(module);
        box.size = Vec(450, 380);
        setPanel(createPanel(asset::plugin(pluginInstance, "res/LINGO.svg")));

        float leftX = 35;
        
        addChild(new BlackLabel(Vec(leftX, 65), "CLOCK"));
        addInput(createInputCentered<PJ301MPort>(Vec(leftX, 90), module, Lingo::CLOCK_INPUT));

        addChild(new BlackLabel(Vec(leftX, 125), "RUN"));
        addParam(createParamCentered<LEDButton>(Vec(leftX, 150), module, Lingo::RUN_PARAM));
        addChild(createLightCentered<MediumLight<GreenLight>>(Vec(leftX, 150), module, Lingo::RUN_LIGHT));

        addChild(new BlackLabel(Vec(leftX, 185), "RUN CV"));
        addInput(createInputCentered<PJ301MPort>(Vec(leftX, 210), module, Lingo::RUN_INPUT));

        addChild(new BlackLabel(Vec(leftX, 245), "RST"));
        addParam(createParamCentered<VCVButton>(Vec(leftX, 270), module, Lingo::RESET_PARAM));

        addChild(new BlackLabel(Vec(leftX, 305), "RST CV"));
        addInput(createInputCentered<PJ301MPort>(Vec(leftX, 330), module, Lingo::RESET_INPUT));

        addChild(new CharDisplay(Vec(205, 25), module));

        ui::ScrollWidget* scrollWidget = new ui::ScrollWidget();
        scrollWidget->box.pos = Vec(85, 75);
        scrollWidget->box.size = Vec(270, 280);

        LingoTextField* textField = new LingoTextField();
        textField->module = module;
        textField->box.size = Vec(270, 280); 

        scrollWidget->container->addChild(textField);
        addChild(scrollWidget);

        float rightX = 410;
        
        addChild(new BlackLabel(Vec(rightX, 20), "WEIGHT"));
        addParam(createParamCentered<RoundBlackKnob>(Vec(rightX, 50), module, Lingo::WEIGHT_PARAM));
        addInput(createInputCentered<PJ301MPort>(Vec(rightX, 80), module, Lingo::WEIGHT_INPUT)); 

        addChild(new BlackLabel(Vec(rightX, 100), "UPPER V"));
        addParam(createParamCentered<RoundBlackKnob>(Vec(rightX, 130), module, Lingo::UPPER_VOLT_PARAM));
        addInput(createInputCentered<PJ301MPort>(Vec(rightX, 160), module, Lingo::UPPER_VOLT_INPUT)); 

        addChild(new BlackLabel(Vec(rightX, 180), "LOWER V"));
        addParam(createParamCentered<RoundBlackKnob>(Vec(rightX, 210), module, Lingo::LOWER_VOLT_PARAM));
        addInput(createInputCentered<PJ301MPort>(Vec(rightX, 240), module, Lingo::LOWER_VOLT_INPUT)); 

        addChild(new BlackLabel(Vec(rightX, 265), "GATE OUT"));
        addOutput(createOutputCentered<PJ301MPort>(Vec(rightX, 290), module, Lingo::GATE_OUTPUT));

        addChild(new BlackLabel(Vec(rightX, 320), "CV OUT"));
        addOutput(createOutputCentered<PJ301MPort>(Vec(rightX, 345), module, Lingo::CV_OUTPUT));
    }
};

Model* modelLingo = createModel<Lingo, LingoWidget>("LINGO");