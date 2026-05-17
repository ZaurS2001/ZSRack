#include "plugin.hpp"
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <cmath>

// 7-segment bitmasks for 0-9
const uint8_t DIGIT_PATTERNS[10] = { 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F };

struct STPWTCH : Module {
    enum ParamIds {
        START_PARAM,
        STOP_PARAM,
        NUM_PARAMS
    };
    enum InputIds { NUM_INPUTS };
    enum OutputIds { NUM_OUTPUTS };
    enum LightIds { NUM_LIGHTS };

    enum State {
        RESET,
        RUNNING,
        PAUSED,
        STOPPED
    };

    // Use absolute sample counting for 100% precision with the audio engine
    uint64_t elapsedSamples = 0;
    double currentSampleRate = 44100.0;

    State state = RESET;
    NVGcolor ledColor = nvgRGB(255, 0, 0); // Default Red

    dsp::SchmittTrigger startTrigger;
    dsp::SchmittTrigger stopTrigger;

    STPWTCH() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
        configButton(START_PARAM, "Start / Pause");
        configButton(STOP_PARAM, "Stop / Reset");
    }

    void process(const ProcessArgs& args) override {
        currentSampleRate = args.sampleRate;

        // Handle Start / Pause button
        if (startTrigger.process(params[START_PARAM].getValue())) {
            if (state == RESET || state == PAUSED) {
                state = RUNNING;
            } else if (state == RUNNING) {
                state = PAUSED;
            } else if (state == STOPPED) {
                // Pressing start after stopping refreshes digits and counts up
                elapsedSamples = 0;
                state = RUNNING;
            }
        }

        // Handle Stop / Reset button
        if (stopTrigger.process(params[STOP_PARAM].getValue())) {
            if (state == RUNNING) {
                // First press stops it at the current time
                state = STOPPED;
            } else if (state == PAUSED || state == STOPPED) {
                // Second press (or pressing while paused) refreshes/resets
                elapsedSamples = 0;
                state = RESET;
            }
        }

        // Advance exact sample time safely
        if (state == RUNNING) {
            elapsedSamples++;
        }
    }

    // Save/Load Custom LED color & state across patches
    json_t* dataToJson() override {
        json_t* rootJ = json_object();
        json_object_set_new(rootJ, "color_r", json_real(ledColor.r));
        json_object_set_new(rootJ, "color_g", json_real(ledColor.g));
        json_object_set_new(rootJ, "color_b", json_real(ledColor.b));
        json_object_set_new(rootJ, "samples", json_integer(elapsedSamples));
        json_object_set_new(rootJ, "state", json_integer(state));
        return rootJ;
    }

    void dataFromJson(json_t* rootJ) override {
        json_t* rJ = json_object_get(rootJ, "color_r");
        if (rJ) ledColor.r = json_number_value(rJ);
        json_t* gJ = json_object_get(rootJ, "color_g");
        if (gJ) ledColor.g = json_number_value(gJ);
        json_t* bJ = json_object_get(rootJ, "color_b");
        if (bJ) ledColor.b = json_number_value(bJ);
        
        json_t* sJ = json_object_get(rootJ, "samples");
        if (sJ) elapsedSamples = json_integer_value(sJ);
        
        json_t* stJ = json_object_get(rootJ, "state");
        if (stJ) state = (State)json_integer_value(stJ);
    }
};

// Custom NanoVG Display Widget
struct STPWTCHDisplay : TransparentWidget {
    STPWTCH* module;

    void drawSegment(NVGcontext* vg, float sx, float sy, float sw_w, float sw_h, bool on, NVGcolor color) {
        nvgBeginPath(vg);
        nvgRoundedRect(vg, sx, sy, sw_w, sw_h, std::min(sw_w, sw_h) / 2.0f);
        if (!on) color.a = 0.05f; // Dim unlit segments
        nvgFillColor(vg, color);
        nvgFill(vg);
    }

    void drawDigit(NVGcontext* vg, float x, float y, float w, float h, int digit, NVGcolor color) {
        uint8_t pat = DIGIT_PATTERNS[digit];
        float sw = w * 0.18f; // segment thickness
        float sl = (h - 3 * sw) / 2.0f; // segment length

        drawSegment(vg, x + sw, y, w - 2 * sw, sw, pat & (1 << 0), color); // Top
        drawSegment(vg, x + w - sw, y + sw, sw, sl, pat & (1 << 1), color); // Top Right
        drawSegment(vg, x + w - sw, y + 2 * sw + sl, sw, sl, pat & (1 << 2), color); // Bottom Right
        drawSegment(vg, x + sw, y + 2 * sw + 2 * sl, w - 2 * sw, sw, pat & (1 << 3), color); // Bottom
        drawSegment(vg, x, y + 2 * sw + sl, sw, sl, pat & (1 << 4), color); // Bottom Left
        drawSegment(vg, x, y + sw, sw, sl, pat & (1 << 5), color); // Top Left
        drawSegment(vg, x + sw, y + sw + sl, w - 2 * sw, sw, pat & (1 << 6), color); // Middle
    }

    void drawColon(NVGcontext* vg, float x, float y, float h, NVGcolor color, bool on) {
        if (!on) color.a = 0.05f;
        nvgFillColor(vg, color);
        float size = 5.0f;
        nvgBeginPath(vg);
        nvgRect(vg, x - size / 2.0f, y + h * 0.3f, size, size);
        nvgRect(vg, x - size / 2.0f, y + h * 0.65f, size, size);
        nvgFill(vg);
    }

    void drawDot(NVGcontext* vg, float x, float y, float h, NVGcolor color) {
        nvgFillColor(vg, color);
        float size = 5.0f;
        nvgBeginPath(vg);
        nvgRect(vg, x - size / 2.0f, y + h - size, size, size);
        nvgFill(vg);
    }

    // 1. STANDARD DRAW PASS: Renders the background (gets dark with room light)
    void draw(const DrawArgs& args) override {
        nvgBeginPath(args.vg);
        nvgRoundedRect(args.vg, 0, 0, box.size.x, box.size.y, 5);
        nvgFillColor(args.vg, nvgRGB(15, 15, 15));
        nvgFill(args.vg);

        Widget::draw(args);
    }

    // 2. LIGHT LAYER PASS: Renders the glowing LEDs (ignores room light)
    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer == 1) {
            if (!module) return;

            // Calculate time in seconds
            double time = (double)module->elapsedSamples / std::max(1.0, module->currentSampleRate);
            
            int h = (int)(time / 3600.0);
            int m = (int)(std::fmod(time, 3600.0) / 60.0);
            int s = (int)(std::fmod(time, 60.0));
            int ms = (int)(std::fmod(time, 1.0) * 1000.0);

            // Standard configuration formatting
            int h1 = (h / 100) % 10, h2 = (h / 10) % 10, h3 = h % 10;
            int m1 = (m / 10) % 10, m2 = m % 10;
            int s1 = (s / 10) % 10, s2 = s % 10;
            int ms1 = (ms / 100) % 10, ms2 = (ms / 10) % 10, ms3 = ms % 10;

            bool flash = (module->state == STPWTCH::RUNNING) ? (ms < 500) : true;
            NVGcolor color = module->ledColor;

            float x = 20.0f;
            float y = 15.0f;
            float dw = 25.0f;
            float dh = 50.0f;
            float ds = 8.0f;
            float sep = 18.0f;

            // Apply a slight skew/slant for an authentic LED look
            nvgSave(args.vg);
            nvgTransform(args.vg, 1, 0, -0.1f, 1, box.size.y * 0.1f, 0);

            // HHH
            drawDigit(args.vg, x, y, dw, dh, h1, color); x += dw + ds;
            drawDigit(args.vg, x, y, dw, dh, h2, color); x += dw + ds;
            drawDigit(args.vg, x, y, dw, dh, h3, color); x += dw + sep;
            drawColon(args.vg, x - sep / 2.0f, y, dh, color, flash);

            // MM
            drawDigit(args.vg, x, y, dw, dh, m1, color); x += dw + ds;
            drawDigit(args.vg, x, y, dw, dh, m2, color); x += dw + sep;
            drawColon(args.vg, x - sep / 2.0f, y, dh, color, flash);

            // SS
            drawDigit(args.vg, x, y, dw, dh, s1, color); x += dw + ds;
            drawDigit(args.vg, x, y, dw, dh, s2, color); x += dw + sep;
            drawDot(args.vg, x - sep / 2.0f, y, dh, color);

            // mmm
            drawDigit(args.vg, x, y, dw, dh, ms1, color); x += dw + ds;
            drawDigit(args.vg, x, y, dw, dh, ms2, color); x += dw + ds;
            drawDigit(args.vg, x, y, dw, dh, ms3, color);

            nvgRestore(args.vg);
        }

        Widget::drawLayer(args, layer);
    }
};

// Dynamic text rendering for button labels based on State
struct StateLabel : TransparentWidget {
    STPWTCH* module;
    bool isStartBtn;

    void draw(const DrawArgs& args) override {
        if (!module) return;
        std::string text = "";
        
        if (isStartBtn) {
            if (module->state == STPWTCH::RESET || module->state == STPWTCH::STOPPED) text = "START";
            else if (module->state == STPWTCH::RUNNING) text = "PAUSE";
            else if (module->state == STPWTCH::PAUSED) text = "CONTINUE";
        } else {
            if (module->state == STPWTCH::RUNNING) text = "STOP";
            else text = "RESET";
        }

        nvgFontSize(args.vg, 16);
        nvgFontFaceId(args.vg, APP->window->uiFont->handle);
        nvgFillColor(args.vg, nvgRGB(50, 50, 50));
        nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
        nvgText(args.vg, box.size.x / 2.0f, box.size.y / 2.0f, text.c_str(), NULL);
    }
};

// Custom Text Field for RGB Hex Parsing in Context Menu
// FIX: Using `widget::Widget` instead of `ui::Widget`
struct ColorMenuWidget : widget::Widget {
    STPWTCH* module;
    ui::TextField* textField;

    ColorMenuWidget(STPWTCH* module) {
        this->module = module;
        box.size = Vec(200, 36);

        textField = createWidget<ui::TextField>(Vec(9, 4));
        textField->box.size = Vec(60, 23);
        textField->multiline = false;
        
        char buf[10];
        snprintf(buf, sizeof(buf), "#%02X%02X%02X",
                 (int)(module->ledColor.r * 255),
                 (int)(module->ledColor.g * 255),
                 (int)(module->ledColor.b * 255));
        textField->text = buf;
        
        addChild(textField);
    }

    void step() override {
        // FIX: Using `widget::Widget::step()`
        widget::Widget::step();
        
        // Parses dynamically as you type
        if (textField->text.length() >= 7 && textField->text[0] == '#') {
            int r, g, b;
            if (sscanf(textField->text.c_str(), "#%02x%02x%02x", &r, &g, &b) == 3) {
                module->ledColor = nvgRGB(r, g, b);
            }
        }
    }
};

// Optional Quick Preset Colors Item
struct PresetColorItem : ui::MenuItem {
    STPWTCH* module;
    NVGcolor ledColorToSet;
    PresetColorItem(STPWTCH* module, std::string name, NVGcolor color) {
        this->module = module;
        this->text = name;
        this->ledColorToSet = color;
    }
    void onAction(const ActionEvent& e) override {
        module->ledColor = ledColorToSet;
    }
};

struct STPWTCHWidget : ModuleWidget {
    STPWTCHWidget(STPWTCH* module) {
        setModule(module);
        setPanel(createPanel(asset::plugin(pluginInstance, "res/STPWTCH.svg")));

        // Standard Rack Screws
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        // Time Display 
        STPWTCHDisplay* display = createWidget<STPWTCHDisplay>(Vec(30, 90));
        display->box.size = Vec(390, 80);
        display->module = module;
        addChild(display);

        // Buttons
        addParam(createParamCentered<LEDButton>(Vec(150, 270), module, STPWTCH::START_PARAM));
        addParam(createParamCentered<LEDButton>(Vec(300, 270), module, STPWTCH::STOP_PARAM));

        // Start/Pause dynamic label
        StateLabel* startLabel = createWidget<StateLabel>(Vec(100, 225));
        startLabel->box.size = Vec(100, 30);
        startLabel->module = module;
        startLabel->isStartBtn = true;
        addChild(startLabel);

        // Stop/Reset dynamic label
        StateLabel* stopLabel = createWidget<StateLabel>(Vec(250, 225));
        stopLabel->box.size = Vec(100, 30);
        stopLabel->module = module;
        stopLabel->isStartBtn = false;
        addChild(stopLabel);
    }

    void appendContextMenu(ui::Menu* menu) override {
        STPWTCH* module = dynamic_cast<STPWTCH*>(this->module);
        if (!module) return;

        // Color Presets Sub-menu logic
        menu->addChild(new ui::MenuEntry);
        menu->addChild(createMenuLabel("LED Color Presets"));
        menu->addChild(new PresetColorItem(module, "Classic Red", nvgRGB(255, 0, 0)));
        menu->addChild(new PresetColorItem(module, "Bright Green", nvgRGB(0, 255, 0)));
        menu->addChild(new PresetColorItem(module, "Neon Blue", nvgRGB(0, 150, 255)));
        menu->addChild(new PresetColorItem(module, "Amber Yellow", nvgRGB(255, 200, 0)));
        menu->addChild(new PresetColorItem(module, "Ghost White", nvgRGB(240, 240, 255)));

        // Manual Text field wrapped properly
        menu->addChild(new ui::MenuEntry);
        menu->addChild(createMenuLabel("Custom Color (Hex)"));
        menu->addChild(new ColorMenuWidget(module));
    }

    // Rendering our module name using the system font
    void draw(const DrawArgs& args) override {
        ModuleWidget::draw(args);
        
        nvgFontSize(args.vg, 24);
        nvgFontFaceId(args.vg, APP->window->uiFont->handle);
        nvgFillColor(args.vg, nvgRGB(34, 34, 34));
        nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
        nvgTextLetterSpacing(args.vg, 4.0f);
        
        nvgText(args.vg, 225.0f, 360.0f, "STPWTCH", NULL);
    }
};

Model* modelSTPWTCH = createModel<STPWTCH, STPWTCHWidget>("STPWTCH");