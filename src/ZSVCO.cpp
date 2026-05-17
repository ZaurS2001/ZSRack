#include "plugin.hpp"
#include <cmath>
#include <algorithm>

// --- DSP STRUCT (Same as before) ---
struct ZSVCO : Module {
    enum ParamId {
        ATTACK_PARAM, DECAY_PARAM, SUSTAIN_PARAM, RELEASE_PARAM,
        ATTACK_CV_PARAM, DECAY_CV_PARAM, SUSTAIN_CV_PARAM, RELEASE_CV_PARAM,
        FREQ_PARAM,
        PARAMS_LEN
    };
    enum InputId {
        ATTACK_CV_INPUT, DECAY_CV_INPUT, SUSTAIN_CV_INPUT, RELEASE_CV_INPUT,
        VOCT_INPUT, GATE_INPUT,
        INPUTS_LEN
    };
    enum OutputId {
        SINE_OUTPUT, TRIANGLE_OUTPUT, SAW_OUTPUT, SQUARE_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId { LIGHTS_LEN };

    float phase = 0.f;
    float envOut = 0.f;
    enum EnvState { OFF, ATTACK, DECAY, SUSTAIN, RELEASE };
    EnvState state = OFF;
    rack::dsp::SchmittTrigger gateTrigger;

    ZSVCO() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        
        // ADSR 
        configParam(ATTACK_PARAM, 0.f, 1.f, 0.1f, "Attack");
        configParam(DECAY_PARAM, 0.f, 1.f, 0.5f, "Decay");
        configParam(SUSTAIN_PARAM, 0.f, 1.f, 0.5f, "Sustain");
        configParam(RELEASE_PARAM, 0.f, 1.f, 0.2f, "Release");

        // CV Attenuators
        configParam(ATTACK_CV_PARAM, -1.f, 1.f, 0.f, "Attack CV Mod");
        configParam(DECAY_CV_PARAM, -1.f, 1.f, 0.f, "Decay CV Mod");
        configParam(SUSTAIN_CV_PARAM, -1.f, 1.f, 0.f, "Sustain CV Mod");
        configParam(RELEASE_CV_PARAM, -1.f, 1.f, 0.f, "Release CV Mod");

        // Freq
        configParam(FREQ_PARAM, -4.f, 4.f, 0.f, "Frequency Offset", " Octaves");

        configInput(VOCT_INPUT, "V/Oct");
        configInput(GATE_INPUT, "Gate");

        configOutput(SINE_OUTPUT, "Sine");
        configOutput(TRIANGLE_OUTPUT, "Triangle");
        configOutput(SAW_OUTPUT, "Saw");
        configOutput(SQUARE_OUTPUT, "Square");
    }

    void process(const ProcessArgs& args) override {
        // --- ADSR Logic ---
        float gate = inputs[GATE_INPUT].getVoltage();
        bool gated = gate >= 1.f;
        
        if (gateTrigger.process(gate)) state = ATTACK;
        else if (!gated && state != OFF && state != RELEASE) state = RELEASE;

        auto scaleTime =[](float val) { return 0.001f * std::pow(10000.f, rack::math::clamp(val, 0.f, 1.f)); };

        float aVal = params[ATTACK_PARAM].getValue() + params[ATTACK_CV_PARAM].getValue() * (inputs[ATTACK_CV_INPUT].getVoltage() / 10.f);
        float dVal = params[DECAY_PARAM].getValue() + params[DECAY_CV_PARAM].getValue() * (inputs[DECAY_CV_INPUT].getVoltage() / 10.f);
        float sVal = params[SUSTAIN_PARAM].getValue() + params[SUSTAIN_CV_PARAM].getValue() * (inputs[SUSTAIN_CV_INPUT].getVoltage() / 10.f);
        sVal = rack::math::clamp(sVal, 0.f, 1.f);
        float rVal = params[RELEASE_PARAM].getValue() + params[RELEASE_CV_PARAM].getValue() * (inputs[RELEASE_CV_INPUT].getVoltage() / 10.f);

        switch (state) {
            case ATTACK:
                envOut += args.sampleTime / scaleTime(aVal);
                if (envOut >= 1.f) { envOut = 1.f; state = DECAY; }
                break;
            case DECAY:
                envOut -= args.sampleTime / scaleTime(dVal);
                if (envOut <= sVal) { envOut = sVal; state = SUSTAIN; }
                break;
            case SUSTAIN:
                envOut = sVal;
                break;
            case RELEASE:
                envOut -= args.sampleTime / scaleTime(rVal);
                if (envOut <= 0.f) { envOut = 0.f; state = OFF; }
                break;
            case OFF:
                envOut = 0.f;
                break;
        }

        // --- VCO Logic ---
        float pitch = params[FREQ_PARAM].getValue() + inputs[VOCT_INPUT].getVoltage();
        float freq = 261.6256f * std::pow(2.f, pitch);

        phase += freq * args.sampleTime;
        if (phase >= 1.f) phase -= 1.f;

        float sinOut = std::sin(2.f * M_PI * phase);
        float sawOut = 2.f * phase - 1.f;
        float sqrOut = (phase < 0.5f) ? 1.f : -1.f;
        float triOut = (phase < 0.5f) ? (4.f * phase - 1.f) : (3.f - 4.f * phase);

        float level = envOut * 5.f;

        if (outputs[SINE_OUTPUT].isConnected())     outputs[SINE_OUTPUT].setVoltage(sinOut * level);
        if (outputs[TRIANGLE_OUTPUT].isConnected()) outputs[TRIANGLE_OUTPUT].setVoltage(triOut * level);
        if (outputs[SAW_OUTPUT].isConnected())      outputs[SAW_OUTPUT].setVoltage(sawOut * level);
        if (outputs[SQUARE_OUTPUT].isConnected())   outputs[SQUARE_OUTPUT].setVoltage(sqrOut * level);
    }
};

// --- HELPER FOR TEXT ---
struct SimpleLabel : Widget {
    std::string text;
    int fontSize;
    NVGcolor color;

    SimpleLabel(Vec pos, std::string text, int fontSize = 10, NVGcolor color = nvgRGB(0x33, 0x33, 0x33)) {
        this->box.pos = pos;
        this->text = text;
        this->fontSize = fontSize;
        this->color = color;
    }

    void draw(const DrawArgs& args) override {
        nvgFontSize(args.vg, fontSize);
        nvgFillColor(args.vg, color);
        nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
        nvgText(args.vg, 0, 0, text.c_str(), NULL);
    }
};

// --- WIDGET ---
struct ZSVCOWidget : ModuleWidget {
    ZSVCOWidget(ZSVCO* module) {
        setModule(module);
        setPanel(createPanel(asset::plugin(pluginInstance, "res/ZSVCO.svg")));

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        // Define Dark Gray color for white background
        NVGcolor textColor = nvgRGB(0x22, 0x22, 0x22);

        // --- ROW 1: ADSR Labels & Knobs ---
        addChild(new SimpleLabel(Vec(24, 25), "A", 11, textColor));
        addChild(new SimpleLabel(Vec(58, 25), "D", 11, textColor));
        addChild(new SimpleLabel(Vec(92, 25), "S", 11, textColor));
        addChild(new SimpleLabel(Vec(126, 25), "R", 11, textColor));

        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(24, 45), module, ZSVCO::ATTACK_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(58, 45), module, ZSVCO::DECAY_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(92, 45), module, ZSVCO::SUSTAIN_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(126, 45), module, ZSVCO::RELEASE_PARAM));

        // --- ROW 2: CV Trimpots ---
        addParam(createParamCentered<Trimpot>(Vec(24, 80), module, ZSVCO::ATTACK_CV_PARAM));
        addParam(createParamCentered<Trimpot>(Vec(58, 80), module, ZSVCO::DECAY_CV_PARAM));
        addParam(createParamCentered<Trimpot>(Vec(92, 80), module, ZSVCO::SUSTAIN_CV_PARAM));
        addParam(createParamCentered<Trimpot>(Vec(126, 80), module, ZSVCO::RELEASE_CV_PARAM));

        // --- ROW 3: CV Inputs ---
        addInput(createInputCentered<PJ301MPort>(Vec(24, 115), module, ZSVCO::ATTACK_CV_INPUT));
        addInput(createInputCentered<PJ301MPort>(Vec(58, 115), module, ZSVCO::DECAY_CV_INPUT));
        addInput(createInputCentered<PJ301MPort>(Vec(92, 115), module, ZSVCO::SUSTAIN_CV_INPUT));
        addInput(createInputCentered<PJ301MPort>(Vec(126, 115), module, ZSVCO::RELEASE_CV_INPUT));

        // --- CENTER: Frequency ---
        addChild(new SimpleLabel(Vec(75, 150), "FREQUENCY", 11, nvgRGB(30,30,30))); // Orange accent
        
        addParam(createParamCentered<RoundHugeBlackKnob>(Vec(75, 190), module, ZSVCO::FREQ_PARAM));

        // --- V/OCT & GATE ---
        addChild(new SimpleLabel(Vec(40, 245), "V/OCT", 10, textColor));
        addChild(new SimpleLabel(Vec(110, 245), "GATE", 10, textColor));

        addInput(createInputCentered<PJ301MPort>(Vec(40, 265), module, ZSVCO::VOCT_INPUT));
        addInput(createInputCentered<PJ301MPort>(Vec(110, 265), module, ZSVCO::GATE_INPUT));

        // --- BOTTOM: Outputs ---
        addChild(new SimpleLabel(Vec(24, 310), "SIN", 9, textColor));
        addChild(new SimpleLabel(Vec(58, 310), "TRI", 9, textColor));
        addChild(new SimpleLabel(Vec(92, 310), "SAW", 9, textColor));
        addChild(new SimpleLabel(Vec(126, 310), "SQR", 9, textColor));

        addOutput(createOutputCentered<PJ301MPort>(Vec(24, 330), module, ZSVCO::SINE_OUTPUT));
        addOutput(createOutputCentered<PJ301MPort>(Vec(58, 330), module, ZSVCO::TRIANGLE_OUTPUT));
        addOutput(createOutputCentered<PJ301MPort>(Vec(92, 330), module, ZSVCO::SAW_OUTPUT));
        addOutput(createOutputCentered<PJ301MPort>(Vec(126, 330), module, ZSVCO::SQUARE_OUTPUT));

        // --- Footer Branding ---
        addChild(new SimpleLabel(Vec(75, 365), "ZS-VCO", 12, nvgRGB(30, 30, 30)));
    }
};

Model* modelZSVCO = createModel<ZSVCO, ZSVCOWidget>("ZSVCO");