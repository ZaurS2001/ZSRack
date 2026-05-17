#include "plugin.hpp"

// The Module Logic
struct RandModule : Module {
    enum ParamId { RATE_PARAM, MIN_VOLT_PARAM, MAX_VOLT_PARAM, PARAMS_LEN };
    enum InputId { CLOCK_INPUT, INPUTS_LEN };
    enum OutputId { RAND_OUTPUT, OUTPUTS_LEN };
    enum LightId { LIGHTS_LEN };

    // 1. Change state variables to arrays to track up to 16 channels (PORT_MAX_CHANNELS)
    float phase[PORT_MAX_CHANNELS] = {};
    float currentVoltage[PORT_MAX_CHANNELS] = {};
    dsp::SchmittTrigger clockTrigger[PORT_MAX_CHANNELS];

    RandModule() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(RATE_PARAM, -4.f, 4.f, 0.f, "Rate", " Hz", 2.f);
        configParam(MIN_VOLT_PARAM, -10.f, 10.f, 0.f, "Min Voltage", " V");
        configParam(MAX_VOLT_PARAM, -10.f, 10.f, 0.f, "Max Voltage", " V");
        configInput(CLOCK_INPUT, "External Clock");
        configOutput(RAND_OUTPUT, "RAND V/Oct");
    }

    void process(const ProcessArgs& args) override {
        // 2. Determine how many channels to process. 
        // Default to 1 (monophonic) for the internal clock if nothing is connected.
        int channels = 1;
        if (inputs[CLOCK_INPUT].isConnected()) {
            channels = inputs[CLOCK_INPUT].getChannels();
        }

        // 3. Inform the output port how many channels it should broadcast
        outputs[RAND_OUTPUT].setChannels(channels);

        // Fetch parameters once per block to save CPU
        float rate = params[RATE_PARAM].getValue();
        float freq = std::pow(2.f, rate);
        float minV = params[MIN_VOLT_PARAM].getValue();
        float maxV = params[MAX_VOLT_PARAM].getValue();

        // 4. Iterate through every active channel
        for (int c = 0; c < channels; c++) {
            bool triggered = false;

            if (inputs[CLOCK_INPUT].isConnected()) {
                // Check trigger for this specific channel 'c'
                if (clockTrigger[c].process(inputs[CLOCK_INPUT].getVoltage(c))) {
                    triggered = true;
                }
            } else {
                // Process internal clock phase for channel 'c'
                phase[c] += freq * args.sampleTime;
                if (phase[c] >= 1.f) { 
                    phase[c] -= 1.f; 
                    triggered = true; 
                }
            }

            // Generate a new random voltage independently for this channel if triggered
            if (triggered) {
                currentVoltage[c] = minV + (random::uniform() * (maxV - minV));
            }

            // 5. Output the voltage to the specific channel 'c'
            outputs[RAND_OUTPUT].setVoltage(currentVoltage[c], c);
        }
    }
};

// --- Helper Widget to Draw Text ---
struct RandPanelText : TransparentWidget {
    void draw(const DrawArgs& args) override {
        std::shared_ptr<Font> font = APP->window->uiFont;
        if (font) nvgFontFaceId(args.vg, font->handle);
        
        nvgFontSize(args.vg, 12.0f);
        nvgFillColor(args.vg, nvgRGB(30, 30, 30));
        nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);

        nvgText(args.vg, 75, 40, "RATE", NULL);
        nvgText(args.vg, 75, 100, "MIN VOLT", NULL);
        nvgText(args.vg, 75, 160, "MAX VOLT", NULL);
        nvgText(args.vg, 75, 225, "EXT CLOCK", NULL);
        nvgText(args.vg, 75, 295, "RAND OUT", NULL);

        nvgFontSize(args.vg, 18.0f);
        nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);
        nvgText(args.vg, 75, 365, "RAND", NULL);
    }
};

struct RandWidget : ModuleWidget {
    RandWidget(RandModule* module) {
        setModule(module);
        
        setPanel(createPanel(asset::plugin(pluginInstance, "res/RAND.svg")));

        RandPanelText* textLayer = createWidget<RandPanelText>(Vec(0, 0));
        textLayer->box.size = box.size; 
        addChild(textLayer);

        float xCenter = 75.0f;
        addParam(createParamCentered<RoundBlackKnob>(Vec(xCenter, 60), module, RandModule::RATE_PARAM));
        addParam(createParamCentered<RoundBlackKnob>(Vec(xCenter, 120), module, RandModule::MIN_VOLT_PARAM));
        addParam(createParamCentered<RoundBlackKnob>(Vec(xCenter, 180), module, RandModule::MAX_VOLT_PARAM));
        addInput(createInputCentered<PJ301MPort>(Vec(xCenter, 250), module, RandModule::CLOCK_INPUT));
        addOutput(createOutputCentered<PJ301MPort>(Vec(xCenter, 320), module, RandModule::RAND_OUTPUT));

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
    }
};

Model* modelRand = createModel<RandModule, RandWidget>("RAND");