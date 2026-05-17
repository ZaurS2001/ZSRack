#include "plugin.hpp"
#include <vector>
#include <algorithm>
#include <cmath>

using namespace rack;

// --- MODULE ---
struct PaceModule : Module {
    enum ParamIds {
        RATE_PARAM,
        MIN_PARAM,
        MAX_PARAM,
        CNG_PARAM,
        SMOOTH_PARAM,
        NUM_PARAMS
    };
    enum InputIds {
        CV_INPUT,
        MIN_CV_INPUT,
        MAX_CV_INPUT,
        SMOOTH_CV_INPUT,
        CLK_INPUT,
        NUM_INPUTS
    };
    enum OutputIds {
        OUT_OUTPUT,
        GATE_OUTPUT,
        NUM_OUTPUTS
    };
    enum LightIds {
        WAVE_LIGHTS,
        NUM_LIGHTS = WAVE_LIGHTS + 6
    };

    float phase = 0.f;
    float penalty = 0.f;
    int waveType = 0; // 0: SIN, 1: TRI, 2: SAW, 3: SQR, 4: RAND, 5: CUSTOM
    
    float randomVal = 0.f;
    float currentRandomVal = 0.f; // Slewed random value

    // Settings configurable via context menu
    float slowdownMultiplier = 0.2f;
    float recoveryMultiplier = 0.2f;
    float xAmount = 0.2f;
    float yAmount = 0.2f;

    // Shared with Widget for thread-safety
    float currentMouseSpeed = 0.f;

    dsp::SchmittTrigger cngTrigger;
    dsp::SchmittTrigger clkTrigger;
    dsp::PulseGenerator gatePulse;

    // Custom Waveform Nodes
    std::vector<math::Vec> customNodes;

    PaceModule() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
        
        configParam(RATE_PARAM, -10.f, 10.f, 1.f, "LFO Rate", " Hz", 2.f);
        configParam(MIN_PARAM, -10.f, 10.f, -5.f, "Minimum Output", " V");
        configParam(MAX_PARAM, -10.f, 10.f, 5.f, "Maximum Output", " V");
        configParam(SMOOTH_PARAM, 0.f, 1.f, 0.f, "S&H Smooth Amount");
        configParam(CNG_PARAM, 0.f, 1.f, 0.f, "Change Wave Type");
        
        configInput(CV_INPUT, "Rate CV");
        configInput(MIN_CV_INPUT, "Min CV");
        configInput(MAX_CV_INPUT, "Max CV");
        configInput(SMOOTH_CV_INPUT, "Smooth CV");
        configInput(CLK_INPUT, "External Clock / Hard Sync");
        
        configOutput(OUT_OUTPUT, "Modulation Output");
        configOutput(GATE_OUTPUT, "Phase Reset Gate");

        // Initial custom nodes: Top left (0, 1) and Bottom right (1, -1)
        customNodes.push_back(math::Vec(0.f, 1.f));
        customNodes.push_back(math::Vec(1.f, -1.f));
    }

    float evalCustomWave(float p) {
        if (customNodes.empty()) return 0.f;
        if (customNodes.size() == 1) return customNodes[0].y;

        for (size_t i = 0; i < customNodes.size() - 1; i++) {
            if (p >= customNodes[i].x && p <= customNodes[i+1].x) {
                float t = (p - customNodes[i].x) / (customNodes[i+1].x - customNodes[i].x);
                return customNodes[i].y + t * (customNodes[i+1].y - customNodes[i].y);
            }
        }
        return customNodes.back().y;
    }

    float evalWave(int type, float p) {
        switch(type) {
            case 0: return std::sin(p * 2.f * M_PI); // SIN
            case 1: return 2.f * std::abs(2.f * p - 1.f) - 1.f; // TRI
            case 2: return 1.f - 2.f * p; // SAW
            case 3: return p < 0.5f ? 1.f : -1.f; // SQR
            case 4: return currentRandomVal; // RAND (S&H) - Uses the smoothly interpolating value
            case 5: return evalCustomWave(p); // CUSTOM
        }
        return 0.f;
    }

    void process(const ProcessArgs& args) override {
        // Change Wave Type
        if (cngTrigger.process(params[CNG_PARAM].getValue())) {
            waveType = (waveType + 1) % 6;
        }

        // Apply mouse movement penalty
        if (currentMouseSpeed > 0.05f) {
            penalty += (currentMouseSpeed / 10.f) * slowdownMultiplier * args.sampleTime;
        } else {
            penalty -= recoveryMultiplier * args.sampleTime;
        }
        penalty = math::clamp(penalty, 0.f, 1.f);

        // Calculate Rate
        float pitch = params[RATE_PARAM].getValue() + inputs[CV_INPUT].getVoltage();
        pitch = math::clamp(pitch, -10.f, 10.f);
        float baseRate = dsp::exp2_taylor5(pitch);
        float actualRate = baseRate * (1.f - penalty);

        bool phaseReset = false;

        // External Clock Reset
        if (inputs[CLK_INPUT].isConnected()) {
            if (clkTrigger.process(inputs[CLK_INPUT].getVoltage())) {
                phase = 0.f;
                phaseReset = true;
            }
        }

        // Advance phase
        float phaseDelta = actualRate * args.sampleTime;
        phase += phaseDelta;
        
        if (phase >= 1.f) {
            phase -= 1.f;
            phaseReset = true;
        }

        if (phaseReset) {
            randomVal = random::uniform() * 2.f - 1.f; // Roll new Random value
            gatePulse.trigger(1e-2f); // Fire 10ms Gate Output
        }

        // Smooth (Slew) logic for S&H Mode
        float smoothVal = math::clamp(params[SMOOTH_PARAM].getValue() + inputs[SMOOTH_CV_INPUT].getVoltage() * 0.1f, 0.f, 1.f);
        if (waveType == 4) {
            if (smoothVal <= 0.001f) {
                currentRandomVal = randomVal; // Instant stepping
            } else {
                float slewTime = 0.001f + 1.0f * smoothVal; // Slew takes up to 1 second
                currentRandomVal += (randomVal - currentRandomVal) * (args.sampleTime / slewTime);
            }
        } else {
            currentRandomVal = randomVal; // Keep it locked instantly so switching modes behaves nicely
        }

        // Output Gate Processing
        outputs[GATE_OUTPUT].setVoltage(gatePulse.process(args.sampleTime) ? 10.f : 0.f);

        // Evaluate Wave (returns normalized -1.0 to 1.0)
        float waveOut = evalWave(waveType, phase);
        
        // Map to Min/Max Ranges
        float min_val = params[MIN_PARAM].getValue() + inputs[MIN_CV_INPUT].getVoltage();
        float max_val = params[MAX_PARAM].getValue() + inputs[MAX_CV_INPUT].getVoltage();
        
        float mappedOut = min_val + (waveOut + 1.f) * 0.5f * (max_val - min_val);

        outputs[OUT_OUTPUT].setVoltage(mappedOut);

        // Update LEDs
        for (int i = 0; i < 6; i++) {
            lights[WAVE_LIGHTS + i].setBrightness(waveType == i ? 1.f : 0.f);
        }
    }

    // Serialization
    json_t* dataToJson() override {
        json_t* rootJ = json_object();
        
        json_t* nodesJ = json_array();
        for (auto node : customNodes) {
            json_t* nodeJ = json_object();
            json_object_set_new(nodeJ, "x", json_real(node.x));
            json_object_set_new(nodeJ, "y", json_real(node.y));
            json_array_append_new(nodesJ, nodeJ);
        }
        json_object_set_new(rootJ, "customNodes", nodesJ);

        json_object_set_new(rootJ, "slowdownMultiplier", json_real(slowdownMultiplier));
        json_object_set_new(rootJ, "recoveryMultiplier", json_real(recoveryMultiplier));
        json_object_set_new(rootJ, "xAmount", json_real(xAmount));
        json_object_set_new(rootJ, "yAmount", json_real(yAmount));
        json_object_set_new(rootJ, "waveType", json_integer(waveType));

        return rootJ;
    }

    void dataFromJson(json_t* rootJ) override {
        json_t* nodesJ = json_object_get(rootJ, "customNodes");
        if (nodesJ) {
            customNodes.clear();
            size_t i; json_t* nodeJ;
            json_array_foreach(nodesJ, i, nodeJ) {
                float x = json_real_value(json_object_get(nodeJ, "x"));
                float y = json_real_value(json_object_get(nodeJ, "y"));
                customNodes.push_back(math::Vec(x, y));
            }
        }
        
        json_t* smJ = json_object_get(rootJ, "slowdownMultiplier");
        if (smJ) slowdownMultiplier = json_real_value(smJ);
        
        json_t* rmJ = json_object_get(rootJ, "recoveryMultiplier");
        if (rmJ) recoveryMultiplier = json_real_value(rmJ);

        json_t* xaJ = json_object_get(rootJ, "xAmount");
        if (xaJ) xAmount = json_real_value(xaJ);

        json_t* yaJ = json_object_get(rootJ, "yAmount");
        if (yaJ) yAmount = json_real_value(yaJ);

        json_t* wtJ = json_object_get(rootJ, "waveType");
        if (wtJ) waveType = json_integer_value(wtJ);
    }
};

// --- DISPLAY WIDGET ---
struct PaceDisplay : TransparentWidget {
    PaceModule* module;
    int dragNode = -1;
    float lastClickTime = 0.f;
    
    math::Vec localMousePos;
    math::Vec dragOffset;

    math::Vec screenToNode(math::Vec sp) {
        return math::Vec(sp.x / box.size.x, 1.f - 2.f * (sp.y / box.size.y));
    }

    math::Vec nodeToScreen(math::Vec n) {
        return math::Vec(n.x * box.size.x, (1.f - n.y) * 0.5f * box.size.y);
    }

    // Maps absolute voltages (-10V to +10V) to display Y coordinates
    float voltToY(float v) {
        return box.size.y - ((v + 10.f) / 20.f) * box.size.y;
    }

    void onButton(const ButtonEvent& e) override {
        if (e.button == GLFW_MOUSE_BUTTON_LEFT && e.action == GLFW_PRESS && module) {
            localMousePos = e.pos;
            float time = system::getTime();
            
            if (time - lastClickTime < 0.25f) {
                handleDoubleClick(e.pos);
                lastClickTime = 0.f;
                e.consume(this);
                return;
            } else {
                lastClickTime = time;
            }

            dragNode = -1;
            for (size_t i = 0; i < module->customNodes.size(); i++) {
                math::Vec screenNode = nodeToScreen(module->customNodes[i]);
                if (math::Vec(e.pos.x - screenNode.x, e.pos.y - screenNode.y).norm() < 10.f) {
                    dragNode = i;
                    dragOffset = screenNode - e.pos; 
                    e.consume(this);
                    return;
                }
            }
        }
        
        if (e.button == GLFW_MOUSE_BUTTON_LEFT && e.action == GLFW_RELEASE) {
            dragNode = -1;
        }
        Widget::onButton(e);
    }

    void handleDoubleClick(math::Vec pos) {
        math::Vec nodePos = screenToNode(pos);
        for (size_t i = 0; i < module->customNodes.size(); i++) {
            math::Vec screenNode = nodeToScreen(module->customNodes[i]);
            if (math::Vec(pos.x - screenNode.x, pos.y - screenNode.y).norm() < 10.f) {
                if (i == 0 || i == module->customNodes.size() - 1) {
                    module->customNodes[i].y = (i == 0) ? 1.f : -1.f;
                } else {
                    module->customNodes.erase(module->customNodes.begin() + i);
                }
                return;
            }
        }
        module->customNodes.push_back(nodePos);
        std::sort(module->customNodes.begin(), module->customNodes.end(), [](math::Vec a, math::Vec b) { return a.x < b.x; });
    }

    void onDragMove(const DragMoveEvent& e) override {
        if (!module || dragNode == -1) return;
        
        localMousePos = localMousePos + e.mouseDelta;
        math::Vec targetScreenNode = localMousePos + dragOffset;
        math::Vec newNode = screenToNode(targetScreenNode);
        
        newNode.y = math::clamp(newNode.y, -1.f, 1.f);
        int lastIndex = static_cast<int>(module->customNodes.size()) - 1;
        
        if (dragNode == 0 || dragNode == lastIndex) {
            newNode.x = (dragNode == 0) ? 0.f : 1.f; 
        } else {
            float leftX = module->customNodes[dragNode - 1].x + 0.001f;
            float rightX = module->customNodes[dragNode + 1].x - 0.001f;
            newNode.x = math::clamp(newNode.x, leftX, rightX);
        }
        
        module->customNodes[dragNode] = newNode;
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer == 1) { 
            // Display Backing Dim Glow
            nvgBeginPath(args.vg);
            nvgRect(args.vg, 0, 0, box.size.x, box.size.y);
            nvgFillColor(args.vg, nvgRGBA(15, 15, 15, 200));
            nvgFill(args.vg);

            float max_v = 5.f;
            float min_v = -5.f;
            if (module) {
                max_v = module->params[PaceModule::MAX_PARAM].getValue() + module->inputs[PaceModule::MAX_CV_INPUT].getVoltage();
                min_v = module->params[PaceModule::MIN_PARAM].getValue() + module->inputs[PaceModule::MIN_CV_INPUT].getVoltage();
            }
            
            // Map voltages to screen Y
            float maxY = voltToY(max_v);
            float minY = voltToY(min_v);

            // Draw MAX Line
            nvgBeginPath(args.vg);
            nvgMoveTo(args.vg, 0, maxY);
            nvgLineTo(args.vg, box.size.x, maxY);
            nvgStrokeColor(args.vg, nvgRGBA(255, 255, 255, 60)); // Faint White
            nvgStrokeWidth(args.vg, 1.5f);
            nvgStroke(args.vg);

            // Draw MIN Line
            nvgBeginPath(args.vg);
            nvgMoveTo(args.vg, 0, minY);
            nvgLineTo(args.vg, box.size.x, minY);
            nvgStroke(args.vg);

            if (!module) return;

            NVGcolor yellow = nvgRGBA(201, 183, 14, 255);

            // Custom Wave Visuals
            if (module->waveType == 5) {
                nvgBeginPath(args.vg);
                nvgStrokeColor(args.vg, nvgRGBA(201, 183, 14, 60)); // Faint yellow structure
                nvgStrokeWidth(args.vg, 1.0f);
                for (float x = 0; x < box.size.x; x += 1.f) {
                    float p = x / box.size.x;
                    float v = module->evalWave(5, p); // -1 to +1
                    float y = box.size.y * (1.f - v) * 0.5f;
                    if (x == 0) nvgMoveTo(args.vg, x, y);
                    else nvgLineTo(args.vg, x, y);
                }
                nvgStroke(args.vg);

                for (auto node : module->customNodes) {
                    math::Vec sp = nodeToScreen(node);
                    nvgBeginPath(args.vg);
                    nvgCircle(args.vg, sp.x, sp.y, 2.0f);
                    nvgFillColor(args.vg, yellow);
                    nvgFill(args.vg);
                }

                // Phase playhead
                float px = module->phase * box.size.x;
                nvgBeginPath(args.vg);
                nvgMoveTo(args.vg, px, 0);
                nvgLineTo(args.vg, px, box.size.y);
                nvgStrokeColor(args.vg, nvgRGBA(255, 255, 255, 120));
                nvgStrokeWidth(args.vg, 1.f);
                nvgStroke(args.vg);
            }
            
            // Draw Output Wave (Scaled between MIN and MAX)
            nvgBeginPath(args.vg);
            nvgStrokeColor(args.vg, yellow);
            nvgStrokeWidth(args.vg, 1.5f);

            for (float x = 0; x < box.size.x; x += 1.f) {
                float p = (module->waveType != 5) ? std::fmod((x / box.size.x) + module->phase, 1.f) : (x / box.size.x);
                float v = module->evalWave(module->waveType, p); // Normalized -1 to 1
                
                // Scale wave to min_v and max_v
                float mappedV = min_v + (v + 1.f) * 0.5f * (max_v - min_v);
                
                // Map the resulting voltage back to screen coordinates
                float y = voltToY(mappedV);
                
                if (x == 0) nvgMoveTo(args.vg, x, y);
                else nvgLineTo(args.vg, x, y);
            }
            nvgStroke(args.vg);

            // Draw Voltage Range Text in Corners
            std::shared_ptr<Font> font = APP->window->uiFont;
            if (font) {
                nvgFontFaceId(args.vg, font->handle);
                nvgFontSize(args.vg, 10);
                
                nvgTextAlign(args.vg, NVG_ALIGN_RIGHT | NVG_ALIGN_TOP);
                nvgFillColor(args.vg, nvgRGBA(255, 255, 255, 180));
                nvgText(args.vg, box.size.x - 4, 4, string::f("MAX: %.1fV", max_v).c_str(), NULL);
                
                nvgTextAlign(args.vg, NVG_ALIGN_RIGHT | NVG_ALIGN_BOTTOM);
                nvgText(args.vg, box.size.x - 4, box.size.y - 4, string::f("MIN: %.1fV", min_v).c_str(), NULL);
            }
        }
        Widget::drawLayer(args, layer);
    }
};

// --- MENU SLIDER WIDGET ---
struct PaceMenuSlider : MenuItem {
    float* target;
    float minVal, maxVal;

    PaceMenuSlider(std::string name, float* target, float min, float max) {
        this->text = name;
        this->target = target;
        this->minVal = min;
        this->maxVal = max;
    }
    
    void onDragMove(const DragMoveEvent& e) override {
        float delta = (e.mouseDelta.x / 150.f) * (maxVal - minVal);
        *target = math::clamp(*target + delta, minVal, maxVal);
    }
    
    void step() override {
        rightText = string::f("%.2f", *target);
        MenuItem::step();
    }
    
    void draw(const DrawArgs& args) override {
        float progress = (*target - minVal) / (maxVal - minVal);
        nvgBeginPath(args.vg);
        nvgRect(args.vg, 0, 0, box.size.x * progress, box.size.y);
        nvgFillColor(args.vg, nvgRGBA(255, 255, 255, 25));
        nvgFill(args.vg);
        MenuItem::draw(args);
    }
};

// --- PROCEDURAL TEXT LABELS ---
struct PaceLabels : Widget {
    PaceLabels() {
        box.size = Vec(150, 380); 
    }
    
    void draw(const DrawArgs& args) override {
        std::shared_ptr<Font> font = APP->window->uiFont;
        if (font) {
            nvgFontFaceId(args.vg, font->handle);
            nvgFillColor(args.vg, nvgRGBA(0, 0, 0, 255)); // Black Text

            // Main Title
            nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_TOP);
            nvgFontSize(args.vg, 18);
            nvgText(args.vg, 75, 11, "PACE", NULL);

            // Knobs text
            nvgFontSize(args.vg, 10);
            nvgText(args.vg, 30, 35, "MIN", NULL);
            nvgText(args.vg, 75, 35, "RATE", NULL);
            nvgText(args.vg, 120, 35, "MAX", NULL);

            // CV Text
            nvgFontSize(args.vg, 9);
            nvgText(args.vg, 30, 79, "CV", NULL);
            nvgText(args.vg, 75, 79, "CV", NULL);
            nvgText(args.vg, 120, 79, "CV", NULL);
            
            // STH Section
            nvgFontSize(args.vg, 10);
            nvgText(args.vg, 120, 120, "STH", NULL);
            nvgFontSize(args.vg, 9);
            nvgText(args.vg, 120, 163, "CV", NULL);

            // Output / CLK / Gate Text
            nvgFontSize(args.vg, 10);
            nvgText(args.vg, 30, 322, "CLK", NULL);
            nvgText(args.vg, 75, 322, "GATE", NULL);
            nvgText(args.vg, 120, 322, "OUT", NULL);

            // Wave Selection Text
            nvgTextAlign(args.vg, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE);
            nvgText(args.vg, 40, 185, "CNG", NULL);

            // Alignment for Wave Names next to LEDs
            nvgText(args.vg, 35, 130, "SIN", NULL);
            nvgText(args.vg, 35, 145, "TRI", NULL);
            nvgText(args.vg, 35, 160, "SAW", NULL);
            
            nvgText(args.vg, 75, 130, "SQR", NULL);
            nvgText(args.vg, 75, 145, "RND", NULL);
            nvgText(args.vg, 75, 160, "CST", NULL);
        }
    }
};

// --- MODULE WIDGET ---
struct PaceWidget : ModuleWidget {
    math::Vec lastMousePos;
    bool initMouse = false;

    PaceWidget(PaceModule* module) {
        setModule(module);
        
        // Ensure PACE.svg is in your /res/ folder!
        setPanel(createPanel(asset::plugin(pluginInstance, "res/PACE.svg")));

        // Load procedural C++ text over top of the SVG
        addChild(new PaceLabels());

        // Standard Screws
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        // --- TOP ROW ---
        addParam(createParamCentered<RoundBlackKnob>(Vec(30, 62), module, PaceModule::MIN_PARAM));
        addInput(createInputCentered<PJ301MPort>(Vec(30, 100), module, PaceModule::MIN_CV_INPUT));
        
        addParam(createParamCentered<RoundBlackKnob>(Vec(75, 62), module, PaceModule::RATE_PARAM));
        addInput(createInputCentered<PJ301MPort>(Vec(75, 100), module, PaceModule::CV_INPUT));
        
        addParam(createParamCentered<RoundBlackKnob>(Vec(120, 62), module, PaceModule::MAX_PARAM));
        addInput(createInputCentered<PJ301MPort>(Vec(120, 100), module, PaceModule::MAX_CV_INPUT));

        // --- MIDDLE ROW ---
        // CNG Button
        addParam(createParamCentered<LEDButton>(Vec(27, 185), module, PaceModule::CNG_PARAM));
        
        // STH (Smooth) Parameter
        addParam(createParamCentered<RoundBlackKnob>(Vec(120, 145), module, PaceModule::SMOOTH_PARAM));
        addInput(createInputCentered<PJ301MPort>(Vec(120, 185), module, PaceModule::SMOOTH_CV_INPUT));

        // Wave LEDs
        addChild(createLightCentered<SmallLight<GreenLight>>(Vec(25, 130), module, PaceModule::WAVE_LIGHTS + 0));
        addChild(createLightCentered<SmallLight<GreenLight>>(Vec(25, 145), module, PaceModule::WAVE_LIGHTS + 1));
        addChild(createLightCentered<SmallLight<GreenLight>>(Vec(25, 160), module, PaceModule::WAVE_LIGHTS + 2));
        
        addChild(createLightCentered<SmallLight<GreenLight>>(Vec(65, 130), module, PaceModule::WAVE_LIGHTS + 3));
        addChild(createLightCentered<SmallLight<GreenLight>>(Vec(65, 145), module, PaceModule::WAVE_LIGHTS + 4));
        addChild(createLightCentered<SmallLight<GreenLight>>(Vec(65, 160), module, PaceModule::WAVE_LIGHTS + 5));

        // --- DISPLAY ---
        PaceDisplay* display = new PaceDisplay();
        display->box.pos = Vec(10, 210);
        display->box.size = Vec(130, 95);
        display->module = module;
        addChild(display);

        // --- BOTTOM ROW ---
        addInput(createInputCentered<PJ301MPort>(Vec(30, 345), module, PaceModule::CLK_INPUT));
        addOutput(createOutputCentered<PJ301MPort>(Vec(75, 345), module, PaceModule::GATE_OUTPUT));
        addOutput(createOutputCentered<PJ301MPort>(Vec(120, 345), module, PaceModule::OUT_OUTPUT));
    }

    void step() override {
        ModuleWidget::step();
        
        // Track mouse distance in Main UI Thread safely
        if (APP->scene) {
            math::Vec mousePos = APP->scene->mousePos;
            if (!initMouse) { lastMousePos = mousePos; initMouse = true; }
            float dx = mousePos.x - lastMousePos.x;
            float dy = mousePos.y - lastMousePos.y;
            lastMousePos = mousePos;

            PaceModule* m = dynamic_cast<PaceModule*>(module);
            if (m) {
                m->currentMouseSpeed = std::abs(dx) * m->xAmount + std::abs(dy) * m->yAmount;
            }
        }
    }

    void appendContextMenu(Menu* menu) override {
        PaceModule* m = dynamic_cast<PaceModule*>(module);
        if (!m) return;

        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("Pace Settings (Drag to Adjust)"));

        menu->addChild(new PaceMenuSlider("Slowdown Pace", &m->slowdownMultiplier, 0.01f, 10.0f));
        menu->addChild(new PaceMenuSlider("Recovery Pace", &m->recoveryMultiplier, 0.01f, 10.0f));
        menu->addChild(new PaceMenuSlider("X-Axis Amount", &m->xAmount, 0.0f, 5.0f));
        menu->addChild(new PaceMenuSlider("Y-Axis Amount", &m->yAmount, 0.0f, 5.0f));
    }
};

Model* modelPACE = createModel<PaceModule, PaceWidget>("PACE");