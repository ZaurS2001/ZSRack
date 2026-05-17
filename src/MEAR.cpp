#include "plugin.hpp"

// Custom draggable slider for Context Menu with a visual bar
struct MEAR_MenuSlider : MenuItem {
    float* target;
    float minV, maxV;
    float dragVal;

    void onDragStart(const DragStartEvent& e) override {
        dragVal = *target;
    }

    void onDragMove(const DragMoveEvent& e) override {
        // Accumulate drag offsets and clamp
        dragVal += e.mouseDelta.x * (maxV - minV) / 200.f;
        *target = clamp(dragVal, minV, maxV);
    }

    void step() override {
        rightText = string::f("%.1f", *target);
        Widget::step();
    }

    void draw(const DrawArgs& args) override {
        // Draw the visual bar filling up based on the slider value
        float pct = (*target - minV) / (maxV - minV);
        nvgFillColor(args.vg, nvgRGBA(255, 255, 255, 35)); // Semi-transparent white bar
        nvgBeginPath(args.vg);
        nvgRect(args.vg, 0, 0, box.size.x * pct, box.size.y);
        nvgFill(args.vg);

        // Draw standard menu item text/interactions
        MenuItem::draw(args);
    }
};

struct Mear : Module {
    enum ParamIds {
        ENUMS(MACRO_PARAM, 8),
        NUM_PARAMS
    };
    enum InputIds {
        ENUMS(MACRO_CV_INPUT, 8),
        NUM_INPUTS
    };
    enum OutputIds {
        ENUMS(MACRO_OUTPUT, 8),
        NUM_OUTPUTS
    };
    enum LightIds {
        NUM_LIGHTS
    };

    bool firstStep = true;

    // State arrays for 8 channels
    bool isAfraid[8] = {false};
    int fearType[8] = {0};
    float fearTimer[8] = {0.f};
    float fearDuration[8] = {3.f};
    
    // Core memory and threshold calculation
    float savedValue[8] = {0.f};
    float lastTotal[8] = {0.f};
    bool lastExceeds[8] = {false};
    
    // Specific emotion states
    float flinchTarget[8] = {0.f};
    float hyperRate[8] = {0.f};
    float shakeIntensity[8] = {0.f};
    float cryRate[8] = {0.f};
    bool cryExp[8] = {false};

    // Global Settings
    float fearTimeSetting = 3.0f;
    bool randomFearTime = false;
    float slewThreshold = 4.0f; // Updated default to 4

    Mear() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
        for (int i = 0; i < 8; i++) {
            configParam(MACRO_PARAM + i, 0.f, 1.f, 0.5f, string::f("Macro %d", i + 1));
            configInput(MACRO_CV_INPUT + i, string::f("Macro %d CV", i + 1));
            configOutput(MACRO_OUTPUT + i, string::f("Macro %d Output", i + 1));
        }
    }

    void process(const ProcessArgs& args) override {
        // Prevent immediate jumping/freaking out when the module is dropped into the rack
        if (firstStep) {
            for (int i = 0; i < 8; i++) {
                lastTotal[i] = params[MACRO_PARAM + i].getValue() + (inputs[MACRO_CV_INPUT + i].getVoltage() / 10.f);
            }
            firstStep = false;
            return;
        }

        for (int i = 0; i < 8; i++) {
            float knobVal = params[MACRO_PARAM + i].getValue();
            float cv = inputs[MACRO_CV_INPUT + i].getVoltage() / 10.f;
            float currentTotal = knobVal + cv;
            
            // Slew Rate (units per second)
            float slew = std::abs(currentTotal - lastTotal[i]) * args.sampleRate;
            bool exceeds = slew > slewThreshold;

            if (!isAfraid[i]) {
                // If the parameter crossed the abuse threshold, 50% chance to become 'scared'
                if (exceeds && !lastExceeds[i]) {
                    if (random::uniform() < 0.5f) {
                        isAfraid[i] = true;
                        fearTimer[i] = 0.f;
                        fearDuration[i] = randomFearTime ? (random::uniform() * (fearTimeSetting - 1.f) + 1.f) : fearTimeSetting;
                        
                        fearType[i] = (int)(random::uniform() * 5.f);
                        if (fearType[i] >= 5) fearType[i] = 4; // Safety clamp
                        
                        savedValue[i] = knobVal; // Remember user's setting to restore later

                        // Setup individual behavior profiles
                        switch (fearType[i]) {
                            case 0: // Flinch
                                flinchTarget[i] = clamp(savedValue[i] + (random::uniform() > 0.5f ? 0.3f : -0.3f), 0.f, 1.f); 
                                break;
                            case 1: // Hyperventilation (10Hz to 30Hz)
                                hyperRate[i] = 10.f + random::uniform() * 20.f; 
                                break;
                            case 2: // Shaking uncontrollably (depends on tweak harshness)
                                shakeIntensity[i] = clamp(slew * 0.005f, 0.02f, 0.15f); 
                                break;
                            case 3: // Crying (fading out)
                                cryExp[i] = random::uniform() > 0.5f;
                                cryRate[i] = savedValue[i] / fearDuration[i]; // Used if linear chosen
                                break;
                            case 4: // Disabling/Locked
                                break; 
                        }
                    }
                }
                lastExceeds[i] = exceeds;
                
                // Normal output behavior (scaling the 0..1 result out to 0..10V CV mapping range)
                if (!isAfraid[i]) { 
                    float outVal = clamp(currentTotal, 0.f, 1.f);
                    outputs[MACRO_OUTPUT + i].setVoltage(outVal * 10.f);
                    lastTotal[i] = currentTotal;
                }
            }

            // Applying fear behavior
            if (isAfraid[i]) {
                fearTimer[i] += args.sampleTime;
                float p = knobVal;

                switch (fearType[i]) {
                    case 0: // Sudden flinch: rapid jump then slow relax
                        if (fearTimer[i] < 0.1f) p += (flinchTarget[i] - p) * 15.f * args.sampleTime;
                        else p += (savedValue[i] - p) * 2.f * args.sampleTime;
                        break;
                    case 1: // Hyperventilation: left/right LFO offset
                        p = savedValue[i] + std::sin(fearTimer[i] * hyperRate[i]) * 0.05f;
                        break;
                    case 2: // Shaking: random noise offset
                        p = savedValue[i] + (random::uniform() - 0.5f) * shakeIntensity[i];
                        break;
                    case 3: // Crying: Linear or Exponential decay
                        if (cryExp[i]) p *= std::exp(-args.sampleTime * (4.605f / fearDuration[i])); // Reaches 1% right at duration limit
                        else p -= cryRate[i] * args.sampleTime;
                        break;
                    case 4: // Locked
                        p = savedValue[i];
                        break;
                }

                p = clamp(p, 0.f, 1.f);
                
                // Emulate physical override to show panic
                if (paramQuantities[MACRO_PARAM + i]) {
                    paramQuantities[MACRO_PARAM + i]->setValue(p);
                }

                outputs[MACRO_OUTPUT + i].setVoltage(p * 10.f); 
                lastTotal[i] = p; // Lock tracking baseline to prevent re-trigger straight out of fear

                // END OF FEAR: Put everything back exactly how it was
                if (fearTimer[i] >= fearDuration[i]) {
                    isAfraid[i] = false;
                    lastExceeds[i] = true; // Demands the user drop below threshold first before becoming afraid again
                    
                    // Physically restore the knob exactly where it was before the fear!
                    if (paramQuantities[MACRO_PARAM + i]) {
                        paramQuantities[MACRO_PARAM + i]->setValue(savedValue[i]);
                    }
                }
            }
        }
    }

    json_t* dataToJson() override {
        json_t* rootJ = json_object();
        json_object_set_new(rootJ, "fearTime", json_real(fearTimeSetting));
        json_object_set_new(rootJ, "randomFear", json_boolean(randomFearTime));
        json_object_set_new(rootJ, "slewThreshold", json_real(slewThreshold));
        return rootJ;
    }

    void dataFromJson(json_t* rootJ) override {
        json_t* ftJ = json_object_get(rootJ, "fearTime");
        if (ftJ) fearTimeSetting = json_real_value(ftJ);
        
        json_t* rfJ = json_object_get(rootJ, "randomFear");
        if (rfJ) randomFearTime = json_boolean_value(rfJ);
        
        json_t* stJ = json_object_get(rootJ, "slewThreshold");
        if (stJ) slewThreshold = json_real_value(stJ);
    }
};

struct MearWidget : ModuleWidget {
    MearWidget(Mear* module) {
        setModule(module);
        setPanel(createPanel(asset::plugin(pluginInstance, "res/Mear.svg")));

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        // C++ Based drawing of text dynamically to leave SVG clean
        struct MearDisplay : Widget {
            void draw(const DrawArgs& args) override {
                std::shared_ptr<Font> font = APP->window->uiFont;
                if (font) {
                    nvgFontFaceId(args.vg, font->handle);
                    nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
                    nvgFillColor(args.vg, nvgRGB(0, 0, 0));

                    // Less cramped layout Labels
                    for (int i = 0; i < 8; i++) {
                        float y = 40 + i * 40; 
                        nvgFontSize(args.vg, 9.f);
                        nvgText(args.vg, 32, y - 16, string::f("M %d", i + 1).c_str(), NULL);
                        nvgText(args.vg, 75, y - 16, "CV", NULL);
                        nvgText(args.vg, 118, y - 16, "OUT", NULL);
                    }

                    // Clean Bottom Title
                    nvgFontSize(args.vg, 20.f);
                    nvgText(args.vg, 75, 362, "MEAR", NULL);
                }
            }
        };

        MearDisplay* display = new MearDisplay();
        display->box.size = box.size;
        addChild(display);

        // Less cramped Layout Map
        for (int i = 0; i < 8; i++) {
            float y = 42 + i * 40;
            addParam(createParamCentered<RoundBlackKnob>(Vec(32, y), module, Mear::MACRO_PARAM + i));
            addInput(createInputCentered<PJ301MPort>(Vec(75, y), module, Mear::MACRO_CV_INPUT + i));
            addOutput(createOutputCentered<PJ301MPort>(Vec(118, y), module, Mear::MACRO_OUTPUT + i));
        }
    }

    void appendContextMenu(Menu* menu) override {
        Mear* module = dynamic_cast<Mear*>(this->module);
        if (!module) return;

        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("Fear Behavior Config"));

        MEAR_MenuSlider* timeSlider = new MEAR_MenuSlider;
        timeSlider->text = "Fear Time (s)";
        timeSlider->target = &module->fearTimeSetting;
        timeSlider->minV = 1.0f;
        timeSlider->maxV = 60.0f;
        menu->addChild(timeSlider);

        struct RandomCheck : MenuItem {
            Mear* mod;
            void onAction(const ActionEvent& e) override {
                mod->randomFearTime = !mod->randomFearTime;
            }
            void step() override {
                rightText = mod->randomFearTime ? "✔" : "";
                Widget::step();
            }
        };
        RandomCheck* rc = new RandomCheck;
        rc->text = "Randomize Fear Duration";
        rc->mod = module;
        menu->addChild(rc);

        MEAR_MenuSlider* slewSlider = new MEAR_MenuSlider;
        slewSlider->text = "Slew Threshold (U/s)";
        slewSlider->target = &module->slewThreshold;
        slewSlider->minV = 0.1f;
        slewSlider->maxV = 20.0f; // Extended range slightly based on updated default 
        menu->addChild(slewSlider);
    }
};

Model* modelMear = createModel<Mear, MearWidget>("MEAR");