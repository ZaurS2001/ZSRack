#include "plugin.hpp"

using namespace rack;

extern Plugin* pluginInstance;

struct BoxState {
    float posX, posY;
    float dirX, dirY;
    
    // Polyphonic Gate System (8 overlapping channels)
    dsp::PulseGenerator gatePulses[8];
    bool isPulsing[8] = {false};
    
    // Timer to keep a channel "occupied" for the allocator, 
    // even after the physical 10ms gate has finished.
    float busyTimer[8] = {0.f}; 

    dsp::SchmittTrigger clkTrigger;
    float clkTime = 0.f;
    float measuredFreq = 0.f;

    // Lowest Available (First Free) Voice Allocator
    void fireBounce() {
        int targetChannel = -1;
        
        // 1. Find the lowest channel that is completely free (timer reached 0)
        for (int c = 0; c < 8; c++) {
            if (busyTimer[c] <= 0.f) {
                targetChannel = c;
                break;
            }
        }
        
        // 2. If all 8 channels are currently busy, steal the one that has been 
        // busy the longest (the one with the lowest remaining timer)
        if (targetChannel == -1) {
            targetChannel = 0;
            float minTime = busyTimer[0];
            for (int c = 1; c < 8; c++) {
                if (busyTimer[c] < minTime) {
                    minTime = busyTimer[c];
                    targetChannel = c;
                }
            }
        }

        // 3. Fire the physical trigger voltage (10ms gate)
        gatePulses[targetChannel].trigger(0.01f); 
        isPulsing[targetChannel] = true;
        
        // 4. Mark this channel as "occupied" for a set duration (e.g., 1.5 seconds).
        // This forces subsequent bounces to migrate to the next available channel, 
        // allowing the previous note's release tail to finish so chords can form.
        busyTimer[targetChannel] = 1.5f; 
    }
};

struct NoSignalModule : Module {
    enum ParamIds {
        RATE_PARAM,       
        RATE_PARAM_END = 4,
        COUNT_PARAM,
        COLLISION_PARAM,
        RESET_PARAM,
        NUM_PARAMS
    };
    enum InputIds {
        RATE_CV_INPUT,    
        RATE_CV_INPUT_END = 4,
        CLK_INPUT,        
        CLK_INPUT_END = 8,
        GLOBAL_CLK_INPUT,
        COUNT_CV_INPUT,
        RESET_INPUT,
        NUM_INPUTS
    };
    enum OutputIds {
        GATE_OUTPUT,      
        GATE_OUTPUT_END = 4,
        NUM_OUTPUTS
    };
    enum LightIds {
        NUM_LIGHTS
    };

    BoxState boxes[4];
    int activeCount = 1;
    dsp::SchmittTrigger resetTrigger;

    const float SCREEN_W = 460.f;
    const float SCREEN_H = 340.f;
    const float BOX_W = 120.f;
    const float BOX_H = 40.f;

    NoSignalModule() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
        
        for(int i = 0; i < 4; i++) {
            configParam(RATE_PARAM + i, 0.f, 10.f, 1.f, string::f("Box %d Rate", i+1), " Hz");
            configInput(RATE_CV_INPUT + i, string::f("Box %d Rate CV", i+1));
            configInput(CLK_INPUT + i, string::f("Box %d CLK", i+1));
            configOutput(GATE_OUTPUT + i, string::f("Box %d Gate", i+1));
        }

        configParam(COUNT_PARAM, 1.f, 4.f, 1.f, "Box Count");
        paramQuantities[COUNT_PARAM]->snapEnabled = true;

        configParam(COLLISION_PARAM, 0.f, 1.f, 1.f, "Box Collision (Up=On, Down=Off)");
        configParam(RESET_PARAM, 0.f, 1.f, 0.f, "Reset Positions");

        configInput(GLOBAL_CLK_INPUT, "Global CLK");
        configInput(COUNT_CV_INPUT, "Count CV");
        configInput(RESET_INPUT, "Reset Trigger In");

        // Initialize positions staggered to prevent immediate lock-up
        for (int i = 0; i < 4; i++) {
            boxes[i].posX = 20.f + i * 50.f;
            boxes[i].posY = 20.f + i * 50.f;
            boxes[i].dirX = (i % 2 == 0) ? 1.f : -1.f;
            boxes[i].dirY = ((i / 2) % 2 == 0) ? 1.f : -1.f;
        }
    }

    void process(const ProcessArgs& args) override {
        // Calculate Active Count
        float countCv = inputs[COUNT_CV_INPUT].isConnected() ? inputs[COUNT_CV_INPUT].getVoltage() : 0.f;
        activeCount = clamp((int)std::round(params[COUNT_PARAM].getValue() + countCv), 1, 4);

        // Process Reset
        if (resetTrigger.process(inputs[RESET_INPUT].getVoltage() + params[RESET_PARAM].getValue())) {
            for (int i = 0; i < 4; i++) {
                boxes[i].posX = 20.f + i * 60.f;
                boxes[i].posY = 20.f + i * 50.f;
            }
        }

        bool collisionEnabled = params[COLLISION_PARAM].getValue() > 0.5f;

        // Process each active box
        for (int i = 0; i < activeCount; i++) {
            float rate = params[RATE_PARAM + i].getValue();

            bool hasLocalClk = inputs[CLK_INPUT + i].isConnected();
            bool hasGlobalClk = inputs[GLOBAL_CLK_INPUT].isConnected();

            if (hasLocalClk || hasGlobalClk) {
                float clkVolts = hasLocalClk ? inputs[CLK_INPUT + i].getVoltage() : inputs[GLOBAL_CLK_INPUT].getVoltage();
                boxes[i].clkTime += args.sampleTime;
                
                if (boxes[i].clkTrigger.process(clkVolts)) {
                    if (boxes[i].clkTime > 0.001f) {
                        boxes[i].measuredFreq = 1.f / boxes[i].clkTime;
                    }
                    boxes[i].clkTime = 0.f;
                }
                if (boxes[i].clkTime > 2.0f) {
                    boxes[i].measuredFreq = 0.f; // Timeout
                }
                rate = boxes[i].measuredFreq;
            } else {
                if (inputs[RATE_CV_INPUT + i].isConnected()) {
                    rate += inputs[RATE_CV_INPUT + i].getVoltage() * 5.f;
                }
            }

            rate = clamp(rate, 0.f, 50.f);
            float speed = rate * (SCREEN_W - BOX_W);

            boxes[i].posX += boxes[i].dirX * speed * args.sampleTime;
            boxes[i].posY += boxes[i].dirY * speed * args.sampleTime;

            bool bounced = false;
            float maxX = SCREEN_W - BOX_W;
            float maxY = SCREEN_H - BOX_H;

            // Wall Collision
            if (boxes[i].posX >= maxX) { boxes[i].posX = maxX; boxes[i].dirX = -1.f; bounced = true; } 
            else if (boxes[i].posX <= 0.f) { boxes[i].posX = 0.f; boxes[i].dirX = 1.f; bounced = true; }

            if (boxes[i].posY >= maxY) { boxes[i].posY = maxY; boxes[i].dirY = -1.f; bounced = true; } 
            else if (boxes[i].posY <= 0.f) { boxes[i].posY = 0.f; boxes[i].dirY = 1.f; bounced = true; }

            if (bounced) {
                boxes[i].fireBounce(); // Triggers polyphonic allocator
            }
        }

        // Process Box-to-Box Collisions (AABB)
        if (collisionEnabled && activeCount > 1) {
            for (int i = 0; i < activeCount; i++) {
                for (int j = i + 1; j < activeCount; j++) {
                    float cx1 = boxes[i].posX + BOX_W / 2.f;
                    float cy1 = boxes[i].posY + BOX_H / 2.f;
                    float cx2 = boxes[j].posX + BOX_W / 2.f;
                    float cy2 = boxes[j].posY + BOX_H / 2.f;

                    if (std::abs(cx1 - cx2) < BOX_W && std::abs(cy1 - cy2) < BOX_H) {
                        float ox = BOX_W - std::abs(cx1 - cx2);
                        float oy = BOX_H - std::abs(cy1 - cy2);

                        if (ox < oy) {
                            // Horizontal collision
                            boxes[i].dirX *= -1.f;
                            boxes[j].dirX *= -1.f;
                            float push = ox / 2.f + 0.1f;
                            if (cx1 < cx2) { boxes[i].posX -= push; boxes[j].posX += push; } 
                            else { boxes[i].posX += push; boxes[j].posX -= push; }
                        } else {
                            // Vertical collision
                            boxes[i].dirY *= -1.f;
                            boxes[j].dirY *= -1.f;
                            float push = oy / 2.f + 0.1f;
                            if (cy1 < cy2) { boxes[i].posY -= push; boxes[j].posY += push; } 
                            else { boxes[i].posY += push; boxes[j].posY -= push; }
                        }

                        // Both boxes fire bounces, allocating to their respective poly channels
                        boxes[i].fireBounce();
                        boxes[j].fireBounce();
                    }
                }
            }
        }

        // --- Set Outputs Polyphonically (8 channels per output) ---
        for (int i = 0; i < 4; i++) {
            outputs[GATE_OUTPUT + i].setChannels(8);

            for (int c = 0; c < 8; c++) {
                // Decrease the busy timer for the voice allocator
                if (boxes[i].busyTimer[c] > 0.f) {
                    boxes[i].busyTimer[c] -= args.sampleTime;
                }

                // Advance the pulse timer for every channel
                boxes[i].isPulsing[c] = boxes[i].gatePulses[c].process(args.sampleTime);
                
                // Set voltage based on whether the box is active AND if this channel is pulsing
                float gateVolts = (i < activeCount && boxes[i].isPulsing[c]) ? 10.f : 0.f;
                outputs[GATE_OUTPUT + i].setVoltage(gateVolts, c);
            }
        }
    }
};

// --- Custom Widget to render panel text natively ---
struct PanelLabels : Widget {
    void draw(const DrawArgs& args) override {
        std::shared_ptr<window::Font> fontRegular;
        std::shared_ptr<window::Font> fontBold;
        std::shared_ptr<Font> font = APP->window->uiFont;
        if (!fontRegular) fontRegular = APP->window->uiFont;
        if (!fontBold) fontBold = APP->window->uiFont;
        if (!fontRegular || !fontBold) return;
        if (font) nvgFontFaceId(args.vg, font->handle);

        // Left side Black text
        nvgFillColor(args.vg, nvgRGB(0, 0, 0));

        // Main title
        nvgFontFaceId(args.vg, fontBold->handle);
        nvgFontSize(args.vg, 16);
        nvgText(args.vg, 650, 362, "NoSignal", NULL);
        
        nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);

        // Headers
        nvgFontSize(args.vg, 12.0f);
        nvgText(args.vg, 30, 42, "RATE", NULL);
        nvgText(args.vg, 75, 42, "CV", NULL);
        nvgText(args.vg, 120, 42, "CLK", NULL);

        // Global row labels
        nvgFontSize(args.vg, 10.0f);
        nvgText(args.vg, 30, 257, "GLB CLK", NULL);
        nvgText(args.vg, 75, 257, "RST IN", NULL);
        nvgText(args.vg, 120, 257, "RST BTN", NULL);

        // System row labels
        nvgText(args.vg, 30, 320, "COUNT", NULL);
        nvgText(args.vg, 75, 320, "CNT CV", NULL);
        nvgText(args.vg, 120, 320, "COLLIDE", NULL);

        // Right side White text
        nvgFillColor(args.vg, nvgRGB(255, 255, 255));
        nvgFontSize(args.vg, 12.0f);
        
        nvgText(args.vg, 680, 65, "GATE 1", NULL);
        nvgText(args.vg, 680, 135, "GATE 2", NULL);
        nvgText(args.vg, 680, 205, "GATE 3", NULL);
        nvgText(args.vg, 680, 275, "GATE 4", NULL);
    }
};

// --- Custom Widget for the Moving DVD Box ---
struct NoSignalDisplay : Widget {
    NoSignalModule* module = nullptr;

    void draw(const DrawArgs& args) override {
        if (!module) return;

        std::shared_ptr<Font> font = APP->window->loadFont(asset::plugin(pluginInstance, "res/DOS.ttf"));

        for (int i = 0; i < module->activeCount; i++) {
            float px = module->boxes[i].posX;
            float py = module->boxes[i].posY;
            float bw = 120.f;
            float bh = 40.f;
            float br = 5.0f; 

            // Left Gradient
            nvgBeginPath(args.vg);
            nvgRoundedRect(args.vg, px, py, bw, bh, br);
            NVGpaint leftPaint = nvgLinearGradient(args.vg, px, py, px + bw/2.f, py, nvgRGB(0x70,0x70,0x70), nvgRGB(0x50,0x50,0x50));
            nvgFillPaint(args.vg, leftPaint);
            nvgFill(args.vg);

            // Right Gradient with clipping mask
            nvgSave(args.vg);
            nvgScissor(args.vg, px + bw/2.f - 1.f, py, bw/2.f + 1.f, bh); 
            nvgBeginPath(args.vg);
            nvgRoundedRect(args.vg, px, py, bw, bh, br);
            NVGpaint rightPaint = nvgLinearGradient(args.vg, px + bw/2.f, py, px + bw, py, nvgRGB(0x50,0x50,0x50), nvgRGB(0x70,0x70,0x70));
            nvgFillPaint(args.vg, rightPaint);
            nvgFill(args.vg);
            nvgRestore(args.vg);

            // Outer white border
            nvgBeginPath(args.vg);
            nvgRoundedRect(args.vg, px, py, bw, bh, br);
            nvgStrokeWidth(args.vg, 2.5f);
            nvgStrokeColor(args.vg, nvgRGB(255, 255, 255));
            nvgStroke(args.vg);

            // Text "NO SIGNAL"
            if (font) nvgFontFaceId(args.vg, font->handle);
            nvgFontSize(args.vg, 12.0f);
            nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
            nvgFillColor(args.vg, nvgRGB(255, 255, 255));
            nvgText(args.vg, px + bw/2.f, py + bh/2.f + 1.f, "NO SIGNAL", NULL);
        }
    }
};

struct NoSignalWidget : ModuleWidget {
    NoSignalWidget(NoSignalModule* module) {
        setModule(module);

        setPanel(createPanel(asset::plugin(pluginInstance, "res/NoSignal.svg")));

        PanelLabels* labels = new PanelLabels();
        labels->box.size = box.size;
        addChild(labels);

        NoSignalDisplay* display = new NoSignalDisplay();
        display->box.pos = Vec(150, 20); 
        display->box.size = Vec(460, 340);
        display->module = module;
        addChild(display);

        // UI Variables
        float xRate = 30.f, xCV = 75.f, xClk = 120.f;
        float yStart = 65.f;
        float ySpace = 45.f;

        // Rows 1 to 4 Inputs
        for (int i = 0; i < 4; i++) {
            addParam(createParamCentered<RoundBlackKnob>(Vec(xRate, yStart + i * ySpace), module, NoSignalModule::RATE_PARAM + i));
            addInput(createInputCentered<PJ301MPort>(Vec(xCV, yStart + i * ySpace), module, NoSignalModule::RATE_CV_INPUT + i));
            addInput(createInputCentered<PJ301MPort>(Vec(xClk, yStart + i * ySpace), module, NoSignalModule::CLK_INPUT + i));
        }

        // Globals Row
        float yGlobals = 280.f;
        addInput(createInputCentered<PJ301MPort>(Vec(xRate, yGlobals), module, NoSignalModule::GLOBAL_CLK_INPUT));
        addInput(createInputCentered<PJ301MPort>(Vec(xCV, yGlobals), module, NoSignalModule::RESET_INPUT));
        addParam(createParamCentered<VCVButton>(Vec(xClk, yGlobals), module, NoSignalModule::RESET_PARAM));

        // System Settings Row
        float ySystem = 340.f;
        addParam(createParamCentered<RoundBlackKnob>(Vec(xRate, ySystem), module, NoSignalModule::COUNT_PARAM));
        addInput(createInputCentered<PJ301MPort>(Vec(xCV, ySystem), module, NoSignalModule::COUNT_CV_INPUT));
        addParam(createParamCentered<CKSS>(Vec(xClk, ySystem), module, NoSignalModule::COLLISION_PARAM));

        // Right side Gate Outputs
        float xOut = 680.f;
        float yOutStart = 90.f;
        float yOutSpace = 70.f;
        
        for (int i = 0; i < 4; i++) {
            addOutput(createOutputCentered<PJ301MPort>(Vec(xOut, yOutStart + i * yOutSpace), module, NoSignalModule::GATE_OUTPUT + i));
        }

        // Hardware Screws
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
    }
};

Model* modelNoSignal = createModel<NoSignalModule, NoSignalWidget>("NoSignal");