#include "plugin.hpp"

// Buffer size for delay line
#define HISTORY_SIZE 262144
#define HISTORY_MASK 0x3FFFF

struct TP_ECHO : Module {
    enum ParamId {
        TIME_L_PARAM,
        TIME_R_PARAM,
        LFO_DEPTH_PARAM,
        LFO_RATE_PARAM,
        FILTER_CUT_PARAM,
        FILTER_RES_PARAM,
        FILTER_TYPE_PARAM,
        FEEDBACK_PARAM,
        MIX_PARAM,
        PARAMS_LEN
    };
    enum InputId {
        LFO_DEPTH_IN,
        LFO_RATE_IN,
        FILTER_CUT_IN,
        CLK_IN, 
        FILTER_RES_IN,
        FEEDBACK_IN,
        IN_L,
        IN_R,
        INPUTS_LEN
    };
    enum OutputId {
        OUT_L,
        OUT_R,
        OUTPUTS_LEN
    };
    enum LightId {
        LIGHTS_LEN
    };

    // Polyphonic states (up to 16 channels)
    float historyL[16][HISTORY_SIZE] = {};
    float historyR[16][HISTORY_SIZE] = {};
    int playhead[16] = {};

    float lfoPhase[16] = {};
    float smoothDelayL[16];
    float smoothDelayR[16];

    // Filter states
    float ic1eq_L[16] = {};
    float ic2eq_L[16] = {};
    float ic1eq_R[16] = {};
    float ic2eq_R[16] = {};

    // Clock trackers
    rack::dsp::SchmittTrigger clockTrigger[16];
    float clockTimer[16] = {};
    float clockInterval[16];

    TP_ECHO() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        
        configParam(TIME_L_PARAM, 1.f, 2000.f, 500.f, "Left Delay Time", " ms");
        configParam(TIME_R_PARAM, 1.f, 2000.f, 500.f, "Right Delay Time", " ms");
        
        configParam(LFO_DEPTH_PARAM, 0.f, 100.f, 0.25f, "LFO Depth", " %");
        configParam(LFO_RATE_PARAM, 0.01f, 20.f, 0.5f, "LFO Rate", " Hz");
        
        configParam(FILTER_CUT_PARAM, 0.f, 1.f, 0.5f, "Filter Cutoff");
        configParam(FILTER_RES_PARAM, 0.f, 2.5f, 0.f, "Filter Resonance");
        
        configSwitch(FILTER_TYPE_PARAM, 0.f, 2.f, 0.f, "Filter Type", {"Lowpass", "Highpass", "Bandpass"});
        
        configParam(FEEDBACK_PARAM, 0.f, 0.98f, 0.5f, "Feedback");
        configParam(MIX_PARAM, 0.f, 1.f, 0.5f, "Dry/Wet Mix");

        configInput(LFO_DEPTH_IN, "LFO Depth CV");
        configInput(LFO_RATE_IN, "LFO Rate CV");
        configInput(FILTER_CUT_IN, "Filter Cutoff CV");
        configInput(CLK_IN, "External Clock");
        configInput(FILTER_RES_IN, "Filter Resonance CV");
        configInput(FEEDBACK_IN, "Feedback CV");
        configInput(IN_L, "Left");
        configInput(IN_R, "Right");

        configOutput(OUT_L, "Left");
        configOutput(OUT_R, "Right");

        // Initialize delays and clock tracking correctly
        for (int c = 0; c < 16; c++) {
            smoothDelayL[c] = 500.f;
            smoothDelayR[c] = 500.f;
            clockInterval[c] = 0.5f;
        }
    }

    // Andrew Simper's SVF
    void processFilter(float& input, float freq, float q, float sampleTime, int type, float& ic1eq, float& ic2eq) {
        float g = std::tan(M_PI * freq * sampleTime);
        float k = 1.0f / q;
        float a1 = 1.0f / (1.0f + g * (g + k));
        float a2 = g * a1;
        float a3 = g * a2;

        float v3 = input - ic2eq;
        float v1 = a1 * ic1eq + a2 * v3;
        float v2 = ic2eq + a2 * ic1eq + a3 * v3;

        ic1eq = 2.0f * v1 - ic1eq;
        ic2eq = 2.0f * v2 - ic2eq;

        if (type == 0) input = v2; // LP
        else if (type == 1) input = input - k * v1 - v2; // HP
        else input = v1; // BP
    }

    void process(const ProcessArgs& args) override {
        // Evaluate maximum required channels
        int channels = 1;
        if (inputs[IN_L].isConnected()) {
            channels = std::max(channels, inputs[IN_L].getChannels());
        }
        if (inputs[IN_R].isConnected()) {
            channels = std::max(channels, inputs[IN_R].getChannels());
        }

        outputs[OUT_L].setChannels(channels);
        outputs[OUT_R].setChannels(channels);

        for (int c = 0; c < channels; c++) {
            float inL = inputs[IN_L].getPolyVoltage(c);
            float inR = inputs[IN_R].isConnected() ? inputs[IN_R].getPolyVoltage(c) : inL;

            // Clock Sync logic
            if (inputs[CLK_IN].isConnected()) {
                if (clockTrigger[c].process(inputs[CLK_IN].getPolyVoltage(c))) {
                    clockInterval[c] = clockTimer[c];
                    clockTimer[c] = 0.f;
                }
                clockTimer[c] += args.sampleTime;
            }

            // LFO Pitch Modulation
            float rate = clamp(params[LFO_RATE_PARAM].getValue() + inputs[LFO_RATE_IN].getPolyVoltage(c), 0.1f, 20.f);
            lfoPhase[c] += rate * args.sampleTime;
            if (lfoPhase[c] >= 1.f) lfoPhase[c] -= 1.f;
            float lfoVal = std::sin(2.f * M_PI * lfoPhase[c]);

            float depth = clamp(params[LFO_DEPTH_PARAM].getValue() + inputs[LFO_DEPTH_IN].getPolyVoltage(c) * 5.f, 0.f, 500.f);

            // Delay Time
            float baseL = params[TIME_L_PARAM].getValue();
            float baseR = params[TIME_R_PARAM].getValue();
            
            if (inputs[CLK_IN].isConnected()) {
                baseL = (baseL / 500.f) * clockInterval[c] * 1000.f;
                baseR = (baseR / 500.f) * clockInterval[c] * 1000.f;
            }

            float targetDelayL = clamp(baseL + lfoVal * depth, 1.f, 5000.f);
            float targetDelayR = clamp(baseR + lfoVal * depth, 1.f, 5000.f);

            smoothDelayL[c] += (targetDelayL - smoothDelayL[c]) * 0.001f;
            smoothDelayR[c] += (targetDelayR - smoothDelayR[c]) * 0.001f;

            // Read Delay
            auto readDelay = [&](float delay_ms, float* history, int ph) {
                float delay_samples = delay_ms * args.sampleRate / 1000.f;
                float read_pos = ph - delay_samples + HISTORY_SIZE;
                int idx1 = ((int)read_pos) & HISTORY_MASK;
                int idx2 = (idx1 + 1) & HISTORY_MASK;
                float frac = read_pos - std::floor(read_pos);
                return history[idx1] * (1.f - frac) + history[idx2] * frac;
            };

            float wetL = readDelay(smoothDelayL[c], historyL[c], playhead[c]);
            float wetR = readDelay(smoothDelayR[c], historyR[c], playhead[c]);

            // Filter Logic
            float cutCv = inputs[FILTER_CUT_IN].getPolyVoltage(c) * 0.1f;
            float freq = 20.f * std::pow(1000.f, clamp(params[FILTER_CUT_PARAM].getValue() + cutCv, 0.f, 1.f));
            
            float resCv = inputs[FILTER_RES_IN].getPolyVoltage(c) * 0.5f;
            float q = clamp(0.707f + params[FILTER_RES_PARAM].getValue() + resCv, 0.1f, 20.f);
            
            int filterType = (int)params[FILTER_TYPE_PARAM].getValue();

            processFilter(wetL, freq, q, args.sampleTime, filterType, ic1eq_L[c], ic2eq_L[c]);
            processFilter(wetR, freq, q, args.sampleTime, filterType, ic1eq_R[c], ic2eq_R[c]);

            // Write Feedback (incorporating external feedback CV logic)
            float fbCv = inputs[FEEDBACK_IN].getPolyVoltage(c) * 0.1f;
            float fb = clamp(params[FEEDBACK_PARAM].getValue() + fbCv, 0.f, 0.98f);
            
            historyL[c][playhead[c]] = clamp(inL + wetL * fb, -10.f, 10.f);
            historyR[c][playhead[c]] = clamp(inR + wetR * fb, -10.f, 10.f);
            playhead[c] = (playhead[c] + 1) & HISTORY_MASK;

            // Output
            float mix = params[MIX_PARAM].getValue();
            outputs[OUT_L].setVoltage(inL * (1.f - mix) + wetL * mix, c);
            outputs[OUT_R].setVoltage(inR * (1.f - mix) + wetR * mix, c);
        }
    }
};

struct TP_ECHOWidget : ModuleWidget {
    TP_ECHOWidget(TP_ECHO* module) {
        setModule(module);
        setPanel(createPanel(asset::plugin(pluginInstance, "res/TP_ECHO.svg")));

        // Add VCV Rack Screws
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        // --- Delay Times ---
        addParam(createParamCentered<RoundBlackKnob>(Vec(40, 45), module, TP_ECHO::TIME_L_PARAM));
        addParam(createParamCentered<RoundBlackKnob>(Vec(110, 45), module, TP_ECHO::TIME_R_PARAM));

        // --- LFO ---
        addParam(createParamCentered<RoundBlackKnob>(Vec(40, 95), module, TP_ECHO::LFO_DEPTH_PARAM));
        addParam(createParamCentered<RoundBlackKnob>(Vec(110, 95), module, TP_ECHO::LFO_RATE_PARAM));
        addInput(createInputCentered<PJ301MPort>(Vec(40, 135), module, TP_ECHO::LFO_DEPTH_IN));
        addInput(createInputCentered<PJ301MPort>(Vec(110, 135), module, TP_ECHO::LFO_RATE_IN));

        // --- Filter ---
        addParam(createParamCentered<RoundBlackKnob>(Vec(25, 185), module, TP_ECHO::FILTER_CUT_PARAM));
        addInput(createInputCentered<PJ301MPort>(Vec(75, 185), module, TP_ECHO::CLK_IN)); // Re-located between knobs
        addParam(createParamCentered<RoundBlackKnob>(Vec(125, 185), module, TP_ECHO::FILTER_RES_PARAM));
        
        addInput(createInputCentered<PJ301MPort>(Vec(25, 225), module, TP_ECHO::FILTER_CUT_IN));
        addInput(createInputCentered<PJ301MPort>(Vec(75, 225), module, TP_ECHO::FEEDBACK_IN)); // Added FEEDBACK CV on the old CLK IN spot
        addInput(createInputCentered<PJ301MPort>(Vec(125, 225), module, TP_ECHO::FILTER_RES_IN));

        // --- Switch, Feedback, Mix ---
        addParam(createParamCentered<CKSSThree>(Vec(25, 270), module, TP_ECHO::FILTER_TYPE_PARAM));
        addParam(createParamCentered<RoundBlackKnob>(Vec(75, 270), module, TP_ECHO::FEEDBACK_PARAM));
        addParam(createParamCentered<RoundBlackKnob>(Vec(125, 270), module, TP_ECHO::MIX_PARAM));

        // --- Stereo I/O ---
        addInput(createInputCentered<PJ301MPort>(Vec(30, 335), module, TP_ECHO::IN_L));
        addInput(createInputCentered<PJ301MPort>(Vec(60, 335), module, TP_ECHO::IN_R));
        addOutput(createOutputCentered<PJ301MPort>(Vec(90, 335), module, TP_ECHO::OUT_L));
        addOutput(createOutputCentered<PJ301MPort>(Vec(120, 335), module, TP_ECHO::OUT_R));
    }

    // Direct UI Text Drawing in C++
    void drawLayer(const DrawArgs& args, int layer) override {
        ModuleWidget::drawLayer(args, layer);

        if (layer == 1) {
            std::shared_ptr<Font> font = APP->window->uiFont;
            if (!font) return;
            
            nvgFontFaceId(args.vg, font->handle);
            nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);

            auto drawText = [&](float x, float y, const char* text, NVGcolor color, float size = 9.0f) {
                nvgFontSize(args.vg, size);
                nvgFillColor(args.vg, color);
                nvgText(args.vg, x, y, text, NULL);
            };

            NVGcolor black = nvgRGB(0, 0, 0); // All text colors converted purely to black 

            // --- Top ---
            drawText(40, 25, "TIME L", black);
            drawText(110, 25, "TIME R", black);

            drawText(40, 75, "LFO DEPTH", black, 8.5f);
            drawText(110, 75, "LFO RATE", black, 8.5f);
            drawText(40, 118, "CV", black, 8.0f);
            drawText(110, 118, "CV", black, 8.0f);

            // --- Filter & Clock ---
            drawText(25, 165, "CUTOFF", black, 8.0f);
            drawText(75, 165, "CLK IN", black, 8.5f); 
            drawText(125, 165, "RES", black, 8.0f);
            
            drawText(25, 208, "CV", black, 8.0f);
            drawText(75, 208, "FB CV", black, 8.0f);
            drawText(125, 208, "CV", black, 8.0f);

            drawText(25, 250, "TYPE", black, 8.0f);
            drawText(75, 250, "FEEDBACK", black, 8.0f);
            drawText(125, 250, "MIX", black, 8.0f);

            // Switch Labels
            nvgTextAlign(args.vg, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE);
            drawText(35, 260, "BP", black, 8.0f);
            drawText(35, 270, "HP", black, 8.0f);
            drawText(35, 280, "LP", black, 8.0f);

            // --- Stereo I/O Labels ---
            nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
            drawText(30, 318, "IN L", black, 8.5f);
            drawText(60, 318, "IN R", black, 8.5f);
            drawText(90, 318, "OUT L", black, 8.5f);
            drawText(120, 318, "OUT R", black, 8.5f);

            // --- Module Title ---
            drawText(75, 365, "TP-ECHO", black, 14.0f);
        }
    }
};

Model* modelTP_ECHO = createModel<TP_ECHO, TP_ECHOWidget>("TP-ECHO");