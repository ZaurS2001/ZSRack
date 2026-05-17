#include "plugin.hpp"
#include "ui/TextField.hpp"
#include "ui/ScrollWidget.hpp"
#include <map>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

using namespace rack;

extern Plugin* pluginInstance;

struct MorseSymbol {
    bool isGate;
    float duration; // units: 1=dot, 2=dash, 3=letter gap, 7=word gap, etc.
    std::string displayString; // text to draw (e.g. ".", "-", " ")
    bool isBreakable; // determines if we can wrap to next line at this token
};

struct MORSE : Module {
    enum ParamIds {
        RATE_PARAM,
        RUN_PARAM,
        RESET_PARAM,
        NUM_PARAMS
    };
    enum InputIds {
        RATE_CV_INPUT,
        EXT_CLK_INPUT,
        RUN_CV_INPUT,
        RESET_CV_INPUT,
        NUM_INPUTS
    };
    enum OutputIds {
        GATE_OUTPUT,
        NUM_OUTPUTS
    };
    enum LightIds {
        RUN_LIGHT,
        NUM_LIGHTS
    };

    std::string inputText = "";
    bool textChanged = false;

    std::vector<MorseSymbol> sequence;
    int currentIndex = 0;
    
    bool running = false;
    float timer = 0.0f;
    
    dsp::SchmittTrigger runTrigger;
    dsp::SchmittTrigger resetTrigger;
    dsp::SchmittTrigger extClkTrigger;

    float extClkTimer = 0.0f;
    float extClkPeriod = 0.0f;
    bool isClocked = false;

    std::map<char, std::string> morseDict = {
        {'A', ".-"}, {'B', "-..."}, {'C', "-.-."}, {'D', "-.."}, {'E', "."},
        {'F', "..-."}, {'G', "--."}, {'H', "...."}, {'I', ".."}, {'J', ".---"},
        {'K', "-.-"}, {'L', ".-.."}, {'M', "--"}, {'N', "-."}, {'O', "---"},
        {'P', ".--."}, {'Q', "--.-"}, {'R', ".-."}, {'S', "..."}, {'T', "-"},
        {'U', "..-"}, {'V', "...-"}, {'W', ".--"}, {'X', "-..-"}, {'Y', "-.--"},
        {'Z', "--.."},
        {'0', "-----"}, {'1', ".----"}, {'2', "..---"}, {'3', "...--"},
        {'4', "....-"}, {'5', "....."}, {'6', "-...."}, {'7', "--..."},
        {'8', "---.."}, {'9', "----."}
    };

    MORSE() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
        
        configParam(RATE_PARAM, 1.0f, 20.0f, 5.0f, "Rate", " Hz");
        configParam(RUN_PARAM, 0.0f, 1.0f, 0.0f, "Run Toggle");
        configParam(RESET_PARAM, 0.0f, 1.0f, 0.0f, "Reset Trigger");

        configInput(RATE_CV_INPUT, "Rate CV");
        configInput(EXT_CLK_INPUT, "External Clock");
        configInput(RUN_CV_INPUT, "Run Toggle CV");
        configInput(RESET_CV_INPUT, "Reset Trigger CV");

        configOutput(GATE_OUTPUT, "Morse Gate");
    }

    void rebuildSequence() {
        sequence.clear();
        bool firstWord = true;
        std::string currentWord = "";

        auto addWord = [&](std::string w) {
            if (w.empty()) return;
            if (!firstWord) {
                sequence.push_back({false, 7.0f, "   ", true}); // WORD_GAP
            }
            firstWord = false;

            for (size_t i = 0; i < w.size(); ++i) {
                char c = std::toupper(w[i]);
                if (morseDict.count(c)) {
                    if (i > 0) {
                        sequence.push_back({false, 3.0f, " ", true}); // LETTER_GAP
                    }
                    std::string code = morseDict[c];
                    for (size_t j = 0; j < code.size(); ++j) {
                        if (j > 0) {
                            sequence.push_back({false, 1.0f, "", false}); // INTRA_GAP
                        }
                        if (code[j] == '.') {
                            sequence.push_back({true, 1.0f, ".", false});
                        } else if (code[j] == '-') {
                            // Dashes output gate twice longer than dots
                            sequence.push_back({true, 2.0f, "-", false}); 
                        }
                    }
                }
            }
        };

        for (char c : inputText) {
            if (c == ' ' || c == '\n' || c == '\t') {
                addWord(currentWord);
                currentWord = "";
            } else {
                currentWord += c;
            }
        }
        addWord(currentWord);

        // Add a Word Gap at the very end to create a natural pause before it loops back
        if (!sequence.empty()) {
            sequence.push_back({false, 7.0f, "   ", true}); 
        }
    }

    void process(const ProcessArgs& args) override {
        if (textChanged) {
            rebuildSequence();
            textChanged = false;
            if (currentIndex >= (int)sequence.size()) {
                currentIndex = sequence.empty() ? 0 : sequence.size() - 1;
            }
        }

        // Run Logic
        if (runTrigger.process(params[RUN_PARAM].getValue() + inputs[RUN_CV_INPUT].getVoltage())) {
            running = !running;
        }
        lights[RUN_LIGHT].setBrightness(running ? 1.0f : 0.0f);

        // Reset Logic
        if (resetTrigger.process(params[RESET_PARAM].getValue() + inputs[RESET_CV_INPUT].getVoltage())) {
            currentIndex = 0;
            timer = 0.0f;
        }

        // External Clock Sync
        extClkTimer += args.sampleTime;
        if (inputs[EXT_CLK_INPUT].isConnected()) {
            if (extClkTrigger.process(inputs[EXT_CLK_INPUT].getVoltage())) {
                extClkPeriod = extClkTimer;
                extClkTimer = 0.0f;
                isClocked = true;
            }
            if (extClkTimer > 5.0f) isClocked = false; // Timeout
        } else {
            isClocked = false;
            extClkTimer = 0.0f;
        }

        float unitDuration;
        if (isClocked && extClkPeriod > 0.001f) {
            unitDuration = extClkPeriod;
        } else {
            float rate = params[RATE_PARAM].getValue();
            rate += inputs[RATE_CV_INPUT].getVoltage() * 2.0f; 
            rate = clamp(rate, 0.1f, 50.0f);
            unitDuration = 1.0f / rate;
        }

        bool gateOut = false;

        // Morse Advancing and Looping
        if (running && !sequence.empty() && currentIndex < (int)sequence.size()) {
            gateOut = sequence[currentIndex].isGate;
            timer += args.sampleTime;

            if (timer >= sequence[currentIndex].duration * unitDuration) {
                timer = 0.0f;
                currentIndex++;
                
                // LOOP BACK TO START
                if (currentIndex >= (int)sequence.size()) {
                    currentIndex = 0;
                }
            }
        }

        outputs[GATE_OUTPUT].setVoltage(gateOut ? 10.0f : 0.0f);
    }

    json_t* dataToJson() override {
        json_t* rootJ = json_object();
        json_object_set_new(rootJ, "text", json_string(inputText.c_str()));
        // Save the run toggle state
        json_object_set_new(rootJ, "running", json_boolean(running));
        return rootJ;
    }

    void dataFromJson(json_t* rootJ) override {
        json_t* textJ = json_object_get(rootJ, "text");
        if (textJ) {
            inputText = json_string_value(textJ);
            textChanged = true;
        }
        // Load the run toggle state
        json_t* runningJ = json_object_get(rootJ, "running");
        if (runningJ) {
            running = json_boolean_value(runningJ);
        }
    }
};

struct MorseDisplay : Widget {
    MORSE* module;
    std::shared_ptr<Font> font;
    
    MorseDisplay() {
        font = APP->window->loadFont(asset::plugin(pluginInstance, "res/DOS.ttf"));
    }

    float getBlockWidth(size_t startIndex, NVGcontext* vg) {
        float bw = 0;
        for (size_t i = startIndex; i < module->sequence.size(); ++i) {
            auto& s = module->sequence[i];
            if (s.isBreakable) break;
            if (!s.displayString.empty()) {
                bw += nvgTextBounds(vg, 0, 0, s.displayString.c_str(), NULL, NULL);
            }
        }
        return bw;
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) return;

        // Background
        nvgBeginPath(args.vg);
        nvgRoundedRect(args.vg, 0, 0, box.size.x, box.size.y, 4.0f);
        nvgFillColor(args.vg, nvgRGB(15, 15, 15));
        nvgFill(args.vg);
        
        // Inner Border Frame
        nvgStrokeColor(args.vg, nvgRGB(50, 50, 50));
        nvgStrokeWidth(args.vg, 2.0f);
        nvgStroke(args.vg);

        if (!module || !font || module->sequence.empty()) return;

        nvgFontFaceId(args.vg, font->handle);

        float padding = 15.0f;
        float maxW = box.size.x - padding * 2;
        float maxH = box.size.y - padding * 2;

        float fontSize = 36.0f;
        float minFontSize = 8.0f;

        struct TokenLayout {
            std::string text;
            float x, y, w;
            bool highlight;
        };
        std::vector<TokenLayout> layout;

        // Auto-fitting Algorithm
        while (fontSize >= minFontSize) {
            nvgFontSize(args.vg, fontSize);
            nvgTextAlign(args.vg, NVG_ALIGN_LEFT | NVG_ALIGN_TOP);
            layout.clear();

            float x = padding;
            float y = padding;
            float lineHeight = fontSize * 1.2f;
            bool fits = true;

            for (size_t i = 0; i < module->sequence.size(); ++i) {
                auto& sym = module->sequence[i];
                if (sym.displayString.empty()) continue;

                float tw = nvgTextBounds(args.vg, 0, 0, sym.displayString.c_str(), NULL, NULL);

                // Word wrap check
                if (!sym.isBreakable && (i == 0 || module->sequence[i-1].isBreakable)) {
                    float bw = getBlockWidth(i, args.vg);
                    if (x + bw > padding + maxW && x > padding) {
                        x = padding;
                        y += lineHeight;
                    }
                }
                
                // Blanket wrap check: If a symbol itself or a wide gap exceeds the boundary
                if (x + tw > padding + maxW && x > padding) {
                    x = padding;
                    y += lineHeight;
                }

                if (y + lineHeight > padding + maxH) {
                    fits = false;
                    break;
                }

                layout.push_back({sym.displayString, x, y, tw, (int)i == module->currentIndex});
                x += tw;
            }

            if (fits || fontSize <= minFontSize) break;
            fontSize -= 2.0f;
        }

        if (layout.empty()) return;

        // Centering Block logic
        float startY = layout.front().y;
        float endY = layout.back().y + (fontSize * 1.2f);
        float totalH = endY - startY;
        float offsetY = std::max(0.0f, (maxH - totalH) / 2.0f);

        for (auto& t : layout) t.y += offsetY;

        for (size_t i = 0; i < layout.size(); ) {
            float lineY = layout[i].y;
            size_t j = i;
            while (j < layout.size() && std::abs(layout[j].y - lineY) < 1.0f) j++;
            
            float actualLineW = layout[j-1].x + layout[j-1].w - layout[i].x;
            float offsetX = std::max(0.0f, (maxW - actualLineW) / 2.0f);

            for (size_t k = i; k < j; ++k) layout[k].x += offsetX;
            i = j;
        }

        // Output Drawing Phase
        nvgFontSize(args.vg, fontSize);
        nvgTextAlign(args.vg, NVG_ALIGN_LEFT | NVG_ALIGN_TOP);
        
        for (auto& t : layout) {
            if (t.highlight) {
                nvgBeginPath(args.vg);
                nvgRect(args.vg, t.x - 2, t.y - 2, t.w + 4, fontSize * 1.2f + 4);
                nvgFillColor(args.vg, nvgRGB(255, 255, 255));
                nvgFill(args.vg);
                nvgFillColor(args.vg, nvgRGB(0, 0, 0)); // Text black when highlighted
            } else {
                nvgFillColor(args.vg, nvgRGB(255, 255, 255)); // Normal text white
            }
            nvgText(args.vg, t.x, t.y, t.text.c_str(), NULL);
        }
    }
};

struct MorseTextField : ui::TextField {
    MORSE* module;

    MorseTextField() {
        this->multiline = true;
    }

    void step() override {
        ui::TextField::step();
        if (module && this->text != module->inputText) {
            module->inputText = this->text;
            module->textChanged = true;
        }
    }

    void draw(const DrawArgs& args) override {
        // Automatically measure bounds and resize self + container to match text
        if (APP->window->uiFont) {
            nvgFontSize(args.vg, 14.0f); 
            nvgFontFaceId(args.vg, APP->window->uiFont->handle);
            nvgTextLineHeight(args.vg, 1.2f);
            float bounds[4];
            nvgTextBoxBounds(args.vg, 0, 0, box.size.x - 12.0f, this->text.c_str(), NULL, bounds);
            float expandedHeight = bounds[3] - bounds[1] + 20.0f; // Height + padding
            box.size.y = std::max(130.0f, expandedHeight);
            
            // Apply this expanding box size to the parent (ScrollWidget's inner container)
            if (this->parent) {
                this->parent->box.size = this->box.size;
            }
        }
        ui::TextField::draw(args);
    }
};

struct MorseTextScrollWidget : ui::ScrollWidget {
    MorseTextScrollWidget() {
        this->box.size = Vec(270, 130);
        // Hide standard internal scrollbars to leave solely the dynamic custom ones
        if (verticalScrollbar) verticalScrollbar->visible = false;
        if (horizontalScrollbar) horizontalScrollbar->visible = false;
    }
    
    void draw(const DrawArgs& args) override {
        // Draw physical text box background behind everything
        nvgBeginPath(args.vg);
        nvgRoundedRect(args.vg, 0, 0, box.size.x, box.size.y, 4.0f);
        nvgFillColor(args.vg, nvgRGB(10, 10, 10)); // Black background
        nvgFill(args.vg);
        
        nvgStrokeColor(args.vg, nvgRGB(100, 100, 100));
        nvgStrokeWidth(args.vg, 1.0f);
        nvgStroke(args.vg);

        // This handles native UI scrolling and draws the text field contents 
        ui::ScrollWidget::draw(args);

        // Draw Dynamic Scrollbar only if container exceeds physical text box frame
        if (container && container->box.size.y > box.size.y) {
            float scrollableRange = container->box.size.y - box.size.y;
            float clampedOffset = std::max(0.0f, std::min(-offset.y, scrollableRange));
            
            float thumbSize = std::max(10.0f, box.size.y * (box.size.y / container->box.size.y));
            float thumbY = (clampedOffset / scrollableRange) * (box.size.y - thumbSize);

            nvgBeginPath(args.vg);
            nvgRoundedRect(args.vg, box.size.x - 6.0f, thumbY, 4.0f, thumbSize, 2.0f);
            nvgFillColor(args.vg, nvgRGBA(255, 255, 255, 128)); // Dimmed white
            nvgFill(args.vg);
        }
    }
};

struct MORSEWidget : ModuleWidget {
    MORSEWidget(MORSE* module) {
        setModule(module);
        setPanel(APP->window->loadSvg(asset::plugin(pluginInstance, "res/MORSE.svg")));

        // Hardware Screws
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        // ====== Left Column ======
        addParam(createParamCentered<RoundBlackKnob>(Vec(45, 60), module, MORSE::RATE_PARAM));
        addInput(createInputCentered<PJ301MPort>(Vec(28, 120), module, MORSE::RATE_CV_INPUT));
        addInput(createInputCentered<PJ301MPort>(Vec(62, 120), module, MORSE::EXT_CLK_INPUT));

        addParam(createParamCentered<LEDButton>(Vec(45, 180), module, MORSE::RUN_PARAM));
        addChild(createLightCentered<MediumLight<RedLight>>(Vec(45, 180), module, MORSE::RUN_LIGHT));
        addInput(createInputCentered<PJ301MPort>(Vec(45, 240), module, MORSE::RUN_CV_INPUT));

        addParam(createParamCentered<TL1105>(Vec(45, 300), module, MORSE::RESET_PARAM));
        addInput(createInputCentered<PJ301MPort>(Vec(45, 350), module, MORSE::RESET_CV_INPUT));

        // ====== Middle Section ======
        MorseDisplay* mDisplay = createWidget<MorseDisplay>(Vec(90, 40));
        mDisplay->box.size = Vec(270, 150);
        mDisplay->module = module;
        addChild(mDisplay);

        MorseTextScrollWidget* mScroll = createWidget<MorseTextScrollWidget>(Vec(90, 200));
        MorseTextField* mTextField = new MorseTextField();
        mTextField->module = module;
        if (module) mTextField->text = module->inputText;
        mTextField->box.size = Vec(270, 130);
        mScroll->container->addChild(mTextField);
        addChild(mScroll);

        // ====== Right Column ======
        addOutput(createOutputCentered<PJ301MPort>(Vec(405, 185), module, MORSE::GATE_OUTPUT));
    }

    // DRAW LABELS DIRECTLY ON TOP OF THE PANEL! No missing bounds or clipping guaranteed!
    void draw(const DrawArgs& args) override {
        // Draw the panel and all widgets (knobs, inputs, displays) first
        ModuleWidget::draw(args);

        // Get standard system font
        std::shared_ptr<Font> font = APP->window->uiFont;
        if (!font) { // Fallback if for some reason uiFont fails initially
            font = APP->window->loadFont(asset::system("res/fonts/Roboto-Regular.ttf"));
        }
        if (!font) return; 

        nvgSave(args.vg);
        nvgFontFaceId(args.vg, font->handle);
        nvgFillColor(args.vg, nvgRGBA(0, 0, 0, 255)); // Solid Black Text
        nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_TOP);

        // Helper to draw text easily by specific absolute pixel coordinates
        auto drawLabel = [&](float x, float y, std::string text, float size) {
            nvgFontSize(args.vg, size);
            nvgText(args.vg, x, y, text.c_str(), NULL);
        };

        // Left 
        drawLabel(45, 35, "RATE", 10.0f);
        drawLabel(28, 100, "RATE CV", 8.0f);
        drawLabel(62, 100, "EXT CLK", 8.0f);
        
        drawLabel(45, 155, "RUN", 10.0f);
        drawLabel(45, 215, "RUN CV", 9.0f);
        
        drawLabel(45, 275, "RESET", 10.0f);
        drawLabel(45, 325, "RST CV", 9.0f);

        // Right
        drawLabel(405, 160, "GATE", 10.0f);
        
        // Bottom Title (450 module width, so 225 is perfect center horizontally)
        drawLabel(225, 355, "MORSE", 16.0f);

        nvgRestore(args.vg);
    }
};

Model* modelMORSE = createModel<MORSE, MORSEWidget>("MORSE");