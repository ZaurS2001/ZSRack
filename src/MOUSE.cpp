#include "plugin.hpp"
#include <GLFW/glfw3.h>
#include <atomic>
#include <algorithm>
#include <cmath>

using namespace rack;

// extern Plugin* pluginInstance;

struct MouseModule : Module {
    enum ParamId {
        SMOOTH_PARAM,
        X_QUANTIZE_PARAM, // Independent X switch
        Y_QUANTIZE_PARAM, // Independent Y switch
        X_MIN_PARAM,
        X_MAX_PARAM,
        Y_MIN_PARAM,
        Y_MAX_PARAM,
        PARAMS_LEN
    };
    enum InputId {
        SMOOTH_CV_INPUT,
        X_CV_INPUT,
        Y_CV_INPUT,
        X_MIN_CV_INPUT,
        X_MAX_CV_INPUT,
        Y_MIN_CV_INPUT,
        Y_MAX_CV_INPUT,
        INPUTS_LEN
    };
    enum OutputId {
        X_OUTPUT,
        X_GATE_OUTPUT, 
        Y_GATE_OUTPUT, 
        Y_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId {
        LIGHTS_LEN
    };

    std::atomic<float> targetX{0.f};
    std::atomic<float> targetY{0.f};
    
    std::atomic<float> currentXMin{0.f};
    std::atomic<float> currentXMax{10.f};
    std::atomic<float> currentYMin{0.f};
    std::atomic<float> currentYMax{10.f};
    
    float smoothX = 0.f;
    float smoothY = 0.f;
    
    std::atomic<float> outX{0.f};
    std::atomic<float> outY{0.f};

    float lastQx = -100.f;
    float lastQy = -100.f;
    dsp::PulseGenerator xGatePulse;
    dsp::PulseGenerator yGatePulse;

    MouseModule() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        
        configParam(SMOOTH_PARAM, 0.f, 1.f, 0.f, "Smoothness");
        configParam(X_QUANTIZE_PARAM, 0.f, 1.f, 0.f, "Quantize X to Chromatic Scale");
        configParam(Y_QUANTIZE_PARAM, 0.f, 1.f, 0.f, "Quantize Y to Chromatic Scale");
        
        configParam(X_MIN_PARAM, -10.f, 10.f, 0.f, "X Axis Minimum", "V");
        configParam(X_MAX_PARAM, -10.f, 10.f, 10.f, "X Axis Maximum", "V");
        configParam(Y_MIN_PARAM, -10.f, 10.f, 0.f, "Y Axis Minimum", "V");
        configParam(Y_MAX_PARAM, -10.f, 10.f, 10.f, "Y Axis Maximum", "V");
        
        configInput(SMOOTH_CV_INPUT, "Smoothness CV");
        configInput(X_CV_INPUT, "X Axis CV Modulation");
        configInput(Y_CV_INPUT, "Y Axis CV Modulation");
        
        configInput(X_MIN_CV_INPUT, "X Min CV");
        configInput(X_MAX_CV_INPUT, "X Max CV");
        configInput(Y_MIN_CV_INPUT, "Y Min CV");
        configInput(Y_MAX_CV_INPUT, "Y Max CV");
        
        configOutput(X_OUTPUT, "X Axis Position");
        configOutput(Y_OUTPUT, "Y Axis Position");
        configOutput(X_GATE_OUTPUT, "X Note Change Gate");
        configOutput(Y_GATE_OUTPUT, "Y Note Change Gate");
    }

    void process(const ProcessArgs& args) override {
        float xMin = params[X_MIN_PARAM].getValue() + (inputs[X_MIN_CV_INPUT].isConnected() ? inputs[X_MIN_CV_INPUT].getVoltage() : 0.f);
        float xMax = params[X_MAX_PARAM].getValue() + (inputs[X_MAX_CV_INPUT].isConnected() ? inputs[X_MAX_CV_INPUT].getVoltage() : 0.f);
        float yMin = params[Y_MIN_PARAM].getValue() + (inputs[Y_MIN_CV_INPUT].isConnected() ? inputs[Y_MIN_CV_INPUT].getVoltage() : 0.f);
        float yMax = params[Y_MAX_PARAM].getValue() + (inputs[Y_MAX_CV_INPUT].isConnected() ? inputs[Y_MAX_CV_INPUT].getVoltage() : 0.f);
        
        currentXMin.store(xMin);
        currentXMax.store(xMax);
        currentYMin.store(yMin);
        currentYMax.store(yMax);

        float smooth = params[SMOOTH_PARAM].getValue();
        if (inputs[SMOOTH_CV_INPUT].isConnected()) smooth += inputs[SMOOTH_CV_INPUT].getVoltage() / 10.f;
        smooth = math::clamp(smooth, 0.f, 1.f);
        float tau = smooth * smooth * 2.0f; 
        
        float tx = targetX.load() * (xMax - xMin) + xMin;
        float ty = targetY.load() * (yMax - yMin) + yMin;

        tx += inputs[X_CV_INPUT].isConnected() ? inputs[X_CV_INPUT].getVoltage() : 0.f;
        ty += inputs[Y_CV_INPUT].isConnected() ? inputs[Y_CV_INPUT].getVoltage() : 0.f;

        if (tau < 0.0001f) {
            smoothX = tx;
            smoothY = ty;
        } else {
            float alpha = 1.0f - std::exp(-args.sampleTime / tau);
            smoothX += (tx - smoothX) * alpha;
            smoothY += (ty - smoothY) * alpha;
        }

        float finalX = smoothX;
        float finalY = smoothY;

        // Independent X Quantization
        if (params[X_QUANTIZE_PARAM].getValue() > 0.5f) {
            finalX = std::round(smoothX * 12.f) / 12.f;
            if (finalX != lastQx) {
                xGatePulse.trigger(0.01f);
                lastQx = finalX;
            }
        } else {
            lastQx = std::round(finalX * 12.f) / 12.f;
        }

        // Independent Y Quantization
        if (params[Y_QUANTIZE_PARAM].getValue() > 0.5f) {
            finalY = std::round(smoothY * 12.f) / 12.f;
            if (finalY != lastQy) {
                yGatePulse.trigger(0.01f);
                lastQy = finalY;
            }
        } else {
            lastQy = std::round(finalY * 12.f) / 12.f;
        }

        outX.store(finalX);
        outY.store(finalY);
        
        outputs[X_OUTPUT].setVoltage(finalX);
        outputs[Y_OUTPUT].setVoltage(finalY);
        outputs[X_GATE_OUTPUT].setVoltage(xGatePulse.process(args.sampleTime) ? 10.f : 0.f);
        outputs[Y_GATE_OUTPUT].setVoltage(yGatePulse.process(args.sampleTime) ? 10.f : 0.f);
    }
};

struct MouseDisplay : TransparentWidget {
    MouseModule* module;

    void step() override {
        TransparentWidget::step();
        
        if (module && APP->window && APP->window->win) {
            GLFWwindow* win = APP->window->win;
            double xpos, ypos;
            glfwGetCursorPos(win, &xpos, &ypos);
            
            int width, height;
            glfwGetWindowSize(win, &width, &height);
            
            if (width > 0 && height > 0) {
                float normX = math::clamp(static_cast<float>(xpos / width), 0.f, 1.f);
                float normY = math::clamp(static_cast<float>(ypos / height), 0.f, 1.f);
                
                module->targetX = normX;
                module->targetY = 1.0f - normY; 
            }
        }
    }

    void draw(const DrawArgs& args) override {
        // Base Black Background
        nvgBeginPath(args.vg);
        nvgRect(args.vg, 0, 0, box.size.x, box.size.y);
        nvgFillColor(args.vg, nvgRGBA(0, 0, 0, 255));
        nvgFill(args.vg);

        if (module) {
            float xMin = module->currentXMin.load();
            float xMax = module->currentXMax.load();
            float yMin = module->currentYMin.load();
            float yMax = module->currentYMax.load();

            bool qx = module->params[MouseModule::X_QUANTIZE_PARAM].getValue() > 0.5f;
            bool qy = module->params[MouseModule::Y_QUANTIZE_PARAM].getValue() > 0.5f;

            // Dynamic 1V/Octave Grid
            nvgBeginPath(args.vg);
            if (qx && xMax != xMin) {
                int startX = std::ceil(std::min(xMin, xMax));
                int endX = std::floor(std::max(xMin, xMax));
                for (int i = startX; i <= endX; i++) {
                    float lx = (i - xMin) / (xMax - xMin) * box.size.x;
                    nvgMoveTo(args.vg, lx, 0);
                    nvgLineTo(args.vg, lx, box.size.y);
                }
            }
            if (qy && yMax != yMin) {
                int startY = std::ceil(std::min(yMin, yMax));
                int endY = std::floor(std::max(yMin, yMax));
                for (int i = startY; i <= endY; i++) {
                    float ly = box.size.y - ((i - yMin) / (yMax - yMin) * box.size.y);
                    nvgMoveTo(args.vg, 0, ly);
                    nvgLineTo(args.vg, box.size.x, ly);
                }
            }
            nvgStrokeColor(args.vg, nvgRGBA(60, 60, 60, 255));
            nvgStrokeWidth(args.vg, 1.0f);
            nvgStroke(args.vg);

            // Draw Corner Static Range Texts
            std::shared_ptr<Font> font = APP->window->uiFont;
            if (font) {
                nvgFontFaceId(args.vg, font->handle);
                nvgFontSize(args.vg, 10);
                nvgFillColor(args.vg, nvgRGBA(150, 150, 150, 255));
                char buf[32];

                snprintf(buf, sizeof(buf), "X:%.1f", xMin);
                nvgTextAlign(args.vg, NVG_ALIGN_LEFT | NVG_ALIGN_TOP);
                nvgText(args.vg, 4, 4, buf, NULL);

                snprintf(buf, sizeof(buf), "X:%.1f", xMax);
                nvgTextAlign(args.vg, NVG_ALIGN_RIGHT | NVG_ALIGN_TOP);
                nvgText(args.vg, box.size.x - 4, 4, buf, NULL);

                snprintf(buf, sizeof(buf), "Y:%.1f", yMin);
                nvgTextAlign(args.vg, NVG_ALIGN_LEFT | NVG_ALIGN_BOTTOM);
                nvgText(args.vg, 4, box.size.y - 4, buf, NULL);

                snprintf(buf, sizeof(buf), "Y:%.1f", yMax);
                nvgTextAlign(args.vg, NVG_ALIGN_RIGHT | NVG_ALIGN_BOTTOM);
                nvgText(args.vg, box.size.x - 4, box.size.y - 4, buf, NULL);
            }
        }
        
        // Inner rim border
        nvgBeginPath(args.vg);
        nvgRect(args.vg, 0, 0, box.size.x, box.size.y);
        nvgStrokeColor(args.vg, nvgRGBA(60, 60, 60, 255));
        nvgStrokeWidth(args.vg, 2.0f);
        nvgStroke(args.vg);

        TransparentWidget::draw(args);
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer == 1 && module) {
            
            // --- NEW: Display Global Backlight Glow ---
            // Emits a soft, translucent light over the whole screen in dim settings
            nvgBeginPath(args.vg);
            nvgRect(args.vg, 0, 0, box.size.x, box.size.y);
            nvgFillColor(args.vg, nvgRGBA(200, 220, 255, 20)); // Soft LCD blue-white backlight
            nvgFill(args.vg);
            
            float xMin = module->currentXMin.load();
            float xMax = module->currentXMax.load();
            float yMin = module->currentYMin.load();
            float yMax = module->currentYMax.load();
            
            float currentOutX = module->outX.load();
            float currentOutY = module->outY.load();

            float radius = 8.f;
            float usableW = box.size.x - radius * 2.f;
            float usableH = box.size.y - radius * 2.f;
            
            float normX = (xMax != xMin) ? (currentOutX - xMin) / (xMax - xMin) : 0.5f;
            float normY = (yMax != yMin) ? (currentOutY - yMin) / (yMax - yMin) : 0.5f;
            
            normX = math::clamp(normX, 0.f, 1.f);
            normY = math::clamp(normY, 0.f, 1.f);

            float cx = radius + normX * usableW;
            float cy = box.size.y - radius - normY * usableH;
            
            // 1. Drop Shadow
            nvgBeginPath(args.vg);
            nvgCircle(args.vg, cx + 2.f, cy + 2.f, radius);
            nvgFillColor(args.vg, nvgRGBA(0, 0, 0, 150));
            nvgFill(args.vg);

            // 2. Yellow Ball
            nvgBeginPath(args.vg);
            nvgCircle(args.vg, cx, cy, radius);
            nvgFillColor(args.vg, nvgRGBA(255, 220, 0, 255));
            nvgFill(args.vg);

            // 3. 3D Highlight
            nvgBeginPath(args.vg);
            nvgCircle(args.vg, cx - 2.f, cy - 2.f, radius * 0.5f);
            NVGpaint paint = nvgRadialGradient(args.vg, cx - 2.f, cy - 2.f, 0.f, radius * 0.6f, 
                                               nvgRGBA(255, 255, 255, 200), 
                                               nvgRGBA(255, 255, 255, 0));
            nvgFillPaint(args.vg, paint);
            nvgFill(args.vg);

            // Realtime Voltage Tracking Text
            std::shared_ptr<Font> font = APP->window->uiFont;
            if (font) {
                nvgFontFaceId(args.vg, font->handle);
                nvgFontSize(args.vg, 10);
                
                char strX[16], strY[16];
                snprintf(strX, sizeof(strX), "%.2f", currentOutX);
                snprintf(strY, sizeof(strY), "%.2f", currentOutY);

                bool nearTopBottom = (cy < 25.f || cy > box.size.y - 25.f);
                bool nearLeftRight = (cx < 25.f || cx > box.size.x - 25.f);

                bool drawHorizontal = nearTopBottom && !nearLeftRight;
                float pad = 12.f;

                auto drawShadowedText = [&](float x, float y, const char* text, int alignment) {
                    nvgTextAlign(args.vg, alignment);
                    nvgFillColor(args.vg, nvgRGBA(0, 0, 0, 200));
                    nvgText(args.vg, x + 1, y + 1, text, NULL);
                    nvgFillColor(args.vg, nvgRGBA(255, 255, 255, 255));
                    nvgText(args.vg, x, y, text, NULL);
                };

                if (drawHorizontal) {
                    drawShadowedText(cx - pad, cy, strX, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE);
                    drawShadowedText(cx + pad, cy, strY, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE);
                } else {
                    float textTopY = cy - pad;
                    float textBotY = cy + pad;
                    int topAlign = NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM;
                    int botAlign = NVG_ALIGN_CENTER | NVG_ALIGN_TOP;
                    
                    if (textTopY < 2.f) {
                        textTopY = cy + 2.f; 
                        topAlign = NVG_ALIGN_CENTER | NVG_ALIGN_TOP;
                    }
                    if (textBotY > box.size.y - 2.f) {
                        textBotY = cy - 2.f;
                        botAlign = NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM;
                    }

                    drawShadowedText(cx, textTopY, strX, topAlign);
                    drawShadowedText(cx, textBotY, strY, botAlign);
                }
            }
        }
        
        TransparentWidget::drawLayer(args, layer);
    }
};

struct MouseModuleWidget : ModuleWidget {
    MouseModuleWidget(MouseModule* module) {
        setModule(module);
        
        box.size = Vec(150, 380);
        setPanel(createPanel(asset::plugin(pluginInstance, "res/MOUSE.svg")));

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        MouseDisplay* display = new MouseDisplay();
        display->box.pos = Vec(15, 35);
        display->box.size = Vec(120, 110);
        display->module = module;
        addChild(display);

        // ROW 1: Range Control Knobs
        addParam(createParamCentered<Trimpot>(Vec(25, 175), module, MouseModule::X_MIN_PARAM));
        addParam(createParamCentered<Trimpot>(Vec(58, 175), module, MouseModule::X_MAX_PARAM));
        addParam(createParamCentered<Trimpot>(Vec(92, 175), module, MouseModule::Y_MIN_PARAM));
        addParam(createParamCentered<Trimpot>(Vec(125, 175), module, MouseModule::Y_MAX_PARAM));

        // ROW 2: Range CV Inputs
        addInput(createInputCentered<PJ301MPort>(Vec(25, 215), module, MouseModule::X_MIN_CV_INPUT));
        addInput(createInputCentered<PJ301MPort>(Vec(58, 215), module, MouseModule::X_MAX_CV_INPUT));
        addInput(createInputCentered<PJ301MPort>(Vec(92, 215), module, MouseModule::Y_MIN_CV_INPUT));
        addInput(createInputCentered<PJ301MPort>(Vec(125, 215), module, MouseModule::Y_MAX_CV_INPUT));

        // ROW 3: Smoothness and Independent Quantize Switches
        addInput(createInputCentered<PJ301MPort>(Vec(25, 260), module, MouseModule::SMOOTH_CV_INPUT));
        addParam(createParamCentered<RoundBlackKnob>(Vec(58, 260), module, MouseModule::SMOOTH_PARAM));
        addParam(createParamCentered<CKSS>(Vec(92, 260), module, MouseModule::X_QUANTIZE_PARAM));
        addParam(createParamCentered<CKSS>(Vec(125, 260), module, MouseModule::Y_QUANTIZE_PARAM));

        // ROW 4: Main CV Modifiers
        addInput(createInputCentered<PJ301MPort>(Vec(45, 305), module, MouseModule::X_CV_INPUT));
        addInput(createInputCentered<PJ301MPort>(Vec(105, 305), module, MouseModule::Y_CV_INPUT));
        
        // ROW 5: Main Outputs (Aligned with 4 columns)
        addOutput(createOutputCentered<PJ301MPort>(Vec(25, 350), module, MouseModule::X_OUTPUT));
        addOutput(createOutputCentered<PJ301MPort>(Vec(58, 350), module, MouseModule::X_GATE_OUTPUT));
        addOutput(createOutputCentered<PJ301MPort>(Vec(92, 350), module, MouseModule::Y_GATE_OUTPUT));
        addOutput(createOutputCentered<PJ301MPort>(Vec(125, 350), module, MouseModule::Y_OUTPUT));
    }

    void draw(const DrawArgs& args) override {
        ModuleWidget::draw(args);

        std::shared_ptr<Font> font = APP->window->uiFont;
        if (!font) return;

        nvgFontFaceId(args.vg, font->handle);
        nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);

        auto drawText = [&](float x, float y, const char* text, int size, NVGcolor color) {
            nvgFontSize(args.vg, size);
            nvgFillColor(args.vg, color);
            nvgText(args.vg, x, y, text, NULL);
        };

        // Row 1 & 2 Labels
        drawText(25, 155, "X MIN", 9, nvgRGB(0, 0, 0));
        drawText(58, 155, "X MAX", 9, nvgRGB(0, 0, 0));
        drawText(92, 155, "Y MIN", 9, nvgRGB(0, 0, 0));
        drawText(125, 155, "Y MAX", 9, nvgRGB(0, 0, 0));
        drawText(25, 195, "CV", 9, nvgRGB(0, 0, 0));
        drawText(58, 195, "CV", 9, nvgRGB(0, 0, 0));
        drawText(92, 195, "CV", 9, nvgRGB(0, 0, 0));
        drawText(125, 195, "CV", 9, nvgRGB(0, 0, 0));

        // Row 3 Labels
        drawText(25, 240, "CV", 10, nvgRGB(0, 0, 0));
        drawText(58, 240, "STH", 10, nvgRGB(0, 0, 0));
        drawText(92, 240, "X Q", 9, nvgRGB(0, 0, 0));
        drawText(125, 240, "Y Q", 9, nvgRGB(0, 0, 0));
        
        // Row 4 Labels
        drawText(45, 285, "X MOD", 10, nvgRGB(0, 0, 0));
        drawText(105, 285, "Y MOD", 10, nvgRGB(0, 0, 0));

        // Row 5 Labels
        drawText(25, 330, "X OUT", 9, nvgRGB(0, 0, 0));
        drawText(58, 330, "X GT", 9, nvgRGB(0, 0, 0));
        drawText(92, 330, "Y GT", 9, nvgRGB(0, 0, 0));
        drawText(125, 330, "Y OUT", 9, nvgRGB(0, 0, 0));
        
        // Bottom Title Plate
        drawText(75, 20, "MOUSE", 16, nvgRGB(0, 0, 0));
    }
};

Model* modelMOUSE = createModel<MouseModule, MouseModuleWidget>("MOUSE");