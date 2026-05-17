#include "plugin.hpp"

using namespace rack;

// ==========================================
// DSP Helper Classes
// ==========================================

struct SVF {
    float ic1eq = 0.f, ic2eq = 0.f;
    float lp = 0.f, bp = 0.f, hp = 0.f;
    void process(float in, float cutoff, float sampleRate) {
        cutoff = clamp(cutoff, 0.01f, sampleRate * 0.49f);
        float g = std::tan(M_PI * cutoff / sampleRate);
        float k = 1.4142f; 
        float a1 = 1.0f / (1.0f + g * (g + k));
        float a2 = g * a1;
        float a3 = g * a2;
        float v3 = in - ic2eq;
        float v1 = a1 * ic1eq + a2 * v3;
        float v2 = ic2eq + a2 * ic1eq + a3 * v3;
        ic1eq = 2.0f * v1 - ic1eq;
        ic2eq = 2.0f * v2 - ic2eq;
        lp = v2;
        bp = v1;
        hp = in - k * v1 - v2;
    }
};

struct PinkNoise {
    float b0=0, b1=0, b2=0, b3=0, b4=0, b5=0, b6=0;
    float process(float white) {
        b0 = 0.99886f * b0 + white * 0.0555179f;
        b1 = 0.99332f * b1 + white * 0.0750759f;
        b2 = 0.96900f * b2 + white * 0.1538520f;
        b3 = 0.86650f * b3 + white * 0.3104856f;
        b4 = 0.55000f * b4 + white * 0.5329522f;
        b5 = -0.7616f * b5 - white * 0.0168980f;
        float pink = b0 + b1 + b2 + b3 + b4 + b5 + b6 + white * 0.5362f;
        b6 = white * 0.115926f;
        return pink * 0.11f;
    }
};

struct BrownNoise {
    float last = 0.f;
    float process(float white) {
        last = (last + (0.02f * white)) / 1.02f;
        return last * 3.5f;
    }
};

struct RandomPulse {
    float timer = 0.f;
    float process(float avg_interval, float width, float sr) {
        if (timer > 0.f) {
            timer -= 1.f;
            return 1.f;
        } else {
            float prob = 1.0f / (avg_interval * sr);
            if (random::uniform() < prob) { 
                timer = width * sr;
                return 1.f;
            }
        }
        return 0.f;
    }
};

struct RandomLFO {
    float current = 0.f;
    float target = 0.f;
    float phase = 0.f;
    
    float process(float sr, float freq) {
        phase += freq / sr;
        if (phase >= 1.f) {
            phase -= 1.f;
            current = target;
            target = (random::uniform() * 2.f) - 1.f;
        }
        float t = phase;
        float smoothT = t * t * (3.0f - 2.0f * t); 
        return current + (target - current) * smoothT;
    }
};

inline float interpolateHermite(float v0, float v1, float v2, float v3, float x) {
    float c0 = v1;
    float c1 = 0.5f * (v2 - v0);
    float c2 = v0 - 2.5f * v1 + 2.0f * v2 - 0.5f * v3;
    float c3 = 1.5f * (v1 - v2) + 0.5f * (v3 - v0);
    return ((c3 * x + c2) * x + c1) * x + c0;
}

// ==========================================
// Module Definition
// ==========================================

struct TapeSim : Module {
    enum ParamIds {
        TAPE_TYPE_PARAM,
        SAT_AMT_PARAM, WOW_RATE_PARAM, WOW_DEPTH_PARAM, WOW_VAR_PARAM,
        FLUTTER_RATE_PARAM, FLUTTER_DEPTH_PARAM, FLUTTER_VAR_PARAM,
        NOISE_TYPE_PARAM, NOISE_VOL_PARAM, DROPOUTS_EN_PARAM,
        DROP_FILTER_VOL_PARAM, DROP_DOUBLING_PARAM, DROP_DELAY_DRIFT_PARAM, DROP_PROB_PARAM,
        NUM_PARAMS
    };
    enum InputIds {
        IN_L_INPUT, IN_R_INPUT,
        SAT_AMT_CV, WOW_RATE_CV, WOW_DEPTH_CV, WOW_VAR_CV,
        FLUTTER_RATE_CV, FLUTTER_DEPTH_CV, FLUTTER_VAR_CV,
        NOISE_VOL_CV,
        DROP_FILTER_VOL_CV, DROP_DOUBLING_CV, DROP_DELAY_DRIFT_CV, DROP_PROB_CV,
        NUM_INPUTS
    };
    enum OutputIds {
        OUT_L_OUTPUT, OUT_R_OUTPUT,
        NUM_OUTPUTS
    };
    enum LightIds {
        NUM_LIGHTS
    };

    SVF baseLp[2], baseHp[2];
    SVF muffleLp[2];
    SVF noiseVhsLp[2];
    SVF dropSmooth[2], chewSmooth[2], trackSmooth[2];
    
    PinkNoise pink[2];
    BrownNoise brown[2];
    RandomPulse dropPulse[2], chewPulse[2], trackPulse[2];
    RandomLFO wowVarRand, flutterVarRand, azRand[2], hissRand;

    float delayBuffer[2][262144] = {{0}};
    uint32_t writePos = 0;
    const uint32_t BUFFER_MASK = 262143;

    float wowPhase = 0.f;
    float flutterPhase = 0.f;
    float humPhase = 0.f;
    float pulsePhase = 0.f;

    TapeSim() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

        // Parameters Configuration
        configParam(TAPE_TYPE_PARAM, 0.f, 1.f, 0.f, "Tape Type", " 0=Cass, 1=VHS");
        configParam(SAT_AMT_PARAM, 0.f, 100.f, 25.f, "Saturation Amount", "%");
        configParam(WOW_RATE_PARAM, 0.1f, 5.f, 1.f, "Wow Rate", " Hz");
        configParam(WOW_DEPTH_PARAM, 0.f, 100.f, 20.f, "Wow Depth", "%");
        configParam(WOW_VAR_PARAM, 0.f, 100.f, 10.f, "Wow Randomness", "%");
        
        configParam(FLUTTER_RATE_PARAM, 5.f, 30.f, 15.f, "Flutter Rate", " Hz");
        configParam(FLUTTER_DEPTH_PARAM, 0.f, 100.f, 20.f, "Flutter Depth", "%");
        configParam(FLUTTER_VAR_PARAM, 0.f, 100.f, 10.f, "Flutter Randomness", "%");
        
        configParam(NOISE_TYPE_PARAM, 0.f, 2.f, 1.f, "Noise Type", " 0=Brw, 1=Pnk, 2=Wht");
        configParam(NOISE_VOL_PARAM, 0.f, 100.f, 10.f, "Noise Volume", "%");
        configParam(DROPOUTS_EN_PARAM, 0.f, 1.f, 1.f, "Enable Dropouts");
        
        configParam(DROP_FILTER_VOL_PARAM, 0.f, 100.f, 50.f, "Filter Drop Depth", "%");
        configParam(DROP_DOUBLING_PARAM, 0.f, 100.f, 30.f, "Doubling Depth", "%");
        configParam(DROP_DELAY_DRIFT_PARAM, 0.f, 100.f, 30.f, "Delay Drift Depth", "%");
        configParam(DROP_PROB_PARAM, 0.f, 100.f, 20.f, "Dropout Probability", "%");

        // Input Tooltips Configuration
        configInput(IN_L_INPUT, "Left Audio In");
        configInput(IN_R_INPUT, "Right Audio In");
        configInput(SAT_AMT_CV, "Saturation Amount CV");
        configInput(WOW_RATE_CV, "Wow Rate CV");
        configInput(WOW_DEPTH_CV, "Wow Depth CV");
        configInput(WOW_VAR_CV, "Wow Randomness CV");
        configInput(FLUTTER_RATE_CV, "Flutter Rate CV");
        configInput(FLUTTER_DEPTH_CV, "Flutter Depth CV");
        configInput(FLUTTER_VAR_CV, "Flutter Randomness CV");
        configInput(NOISE_VOL_CV, "Noise Volume CV");
        configInput(DROP_FILTER_VOL_CV, "Filter Drop Depth CV");
        configInput(DROP_DOUBLING_CV, "Doubling Depth CV");
        configInput(DROP_DELAY_DRIFT_CV, "Delay Drift Depth CV");
        configInput(DROP_PROB_CV, "Dropout Probability CV");

        // Output Tooltips Configuration
        configOutput(OUT_L_OUTPUT, "Left Audio Out");
        configOutput(OUT_R_OUTPUT, "Right Audio Out");
    }

    void process(const ProcessArgs& args) override {
        float sr = args.sampleRate;
        float baseDelay = 0.15f * sr; 
        
        bool isVhs = params[TAPE_TYPE_PARAM].getValue() > 0.5f;
        bool isDropouts = params[DROPOUTS_EN_PARAM].getValue() > 0.5f;
        int noiseType = (int)std::round(params[NOISE_TYPE_PARAM].getValue());

        auto getModVal = [&](int paramId, int cvId, float minV, float maxV) {
            float val = params[paramId].getValue() + (inputs[cvId].isConnected() ? inputs[cvId].getVoltage() * 10.f : 0.f);
            return clamp(val, minV, maxV);
        };

        float satAmt = getModVal(SAT_AMT_PARAM, SAT_AMT_CV, 0.f, 100.f);
        float wRate = getModVal(WOW_RATE_PARAM, WOW_RATE_CV, 0.1f, 5.f);
        float wDepth = getModVal(WOW_DEPTH_PARAM, WOW_DEPTH_CV, 0.f, 100.f);
        float wVar = getModVal(WOW_VAR_PARAM, WOW_VAR_CV, 0.f, 100.f);
        float fRate = getModVal(FLUTTER_RATE_PARAM, FLUTTER_RATE_CV, 5.f, 30.f);
        float fDepth = getModVal(FLUTTER_DEPTH_PARAM, FLUTTER_DEPTH_CV, 0.f, 100.f);
        float fVar = getModVal(FLUTTER_VAR_PARAM, FLUTTER_VAR_CV, 0.f, 100.f);
        float nVol = getModVal(NOISE_VOL_PARAM, NOISE_VOL_CV, 0.f, 100.f);
        float dFiltVol = getModVal(DROP_FILTER_VOL_PARAM, DROP_FILTER_VOL_CV, 0.f, 100.f);
        float dDouble = getModVal(DROP_DOUBLING_PARAM, DROP_DOUBLING_CV, 0.f, 100.f);
        float dDrift = getModVal(DROP_DELAY_DRIFT_PARAM, DROP_DELAY_DRIFT_CV, 0.f, 100.f);
        float dProb = getModVal(DROP_PROB_PARAM, DROP_PROB_CV, 0.f, 100.f);

        float S = satAmt / 100.0f;
        float drive = 1.0f + S * 5.0f;
        float baseLpCut = std::max(18000.f - S * 8000.f, 1000.f);
        float baseHpCut = 20.f + S * 40.f;

        float p = dProb / 100.0f;
        float avgInterval = 50.0f - (p * 49.5f);
        float muffleCutoff = std::max(300.f, 15000.f - (dFiltVol / 100.f) * 14700.f);

        float wSmooth = wowVarRand.process(sr, std::max(0.1f, wRate * 0.5f));
        float fSmooth = flutterVarRand.process(sr, std::max(0.5f, fRate * 0.5f));
        
        float wFreqMod = wRate * (1.0f + (wVar / 100.f) * wSmooth);
        wowPhase += wFreqMod / sr;
        if (wowPhase > 1.f) wowPhase -= 1.f;
        float wLfo = std::sin(2.f * M_PI * wowPhase) * ((wDepth / 100.f) * 0.005f * sr);

        float fFreqMod = fRate * (1.0f + (fVar / 100.f) * fSmooth);
        flutterPhase += fFreqMod / sr;
        if (flutterPhase > 1.f) flutterPhase -= 1.f;
        float fLfo = std::sin(2.f * M_PI * flutterPhase) * ((fDepth / 100.f) * 0.001f * sr);

        // VHS Hum
        humPhase += 50.f / sr;
        if (humPhase > 1.f) humPhase -= 1.f;
        pulsePhase += 25.f / sr;
        if (pulsePhase > 1.f) pulsePhase -= 1.f;

        float hum = std::sin(2.f * M_PI * humPhase) + 0.5f * std::sin(4.f * M_PI * humPhase) + 0.25f * std::sin(6.f * M_PI * humPhase);
        float pulse = std::max(0.8f, (2.f * pulsePhase - 1.f)) - 0.8f;
        float humSignal = (hum * 0.5f + pulse * 5.0f) * 0.015f;
        
        float hissMod = hissRand.process(sr, 2.f);

        for (int ch = 0; ch < 2; ch++) {
            float in = 0.f;
            if (ch == 0) in = inputs[IN_L_INPUT].getVoltage() / 5.f;
            else in = inputs[IN_R_INPUT].isConnected() ? (inputs[IN_R_INPUT].getVoltage() / 5.f) : (inputs[IN_L_INPUT].getVoltage() / 5.f);

            in = std::tanh(in * drive);
            baseLp[ch].process(in, baseLpCut, sr);
            in = baseLp[ch].lp;
            baseHp[ch].process(in, baseHpCut, sr);
            in = baseHp[ch].hp;

            delayBuffer[ch][writePos & BUFFER_MASK] = in;

            float dropEnv = 0.f, chewEnv = 0.f, trackEnv = 0.f;
            if (isDropouts) {
                float rawDrop = dropPulse[ch].process(avgInterval, 0.2f + p * 0.5f, sr);
                float rawChew = chewPulse[ch].process(avgInterval * 1.5f, 1.0f + p * 1.0f, sr);
                float rawTrack = isVhs ? trackPulse[ch].process(avgInterval * 1.2f, 3.0f, sr) : 0.f;
                
                dropSmooth[ch].process(rawDrop, 2.0f, sr);
                chewSmooth[ch].process(rawChew, 2.0f, sr);
                trackSmooth[ch].process(rawTrack, 2.0f, sr);
                
                dropEnv = std::max(0.f, dropSmooth[ch].lp);
                chewEnv = std::max(0.f, chewSmooth[ch].lp);
                trackEnv = std::max(0.f, trackSmooth[ch].lp);
            }

            float azLfo = azRand[ch].process(sr, 0.5f) * ((dDrift / 100.f) * 0.001f * sr);
            float driftSmp = (dDrift / 100.f) * 0.1f * sr;
            float chewDelay = chewEnv * driftSmp;
            float trackingDelay = isVhs ? (trackEnv * (S * 0.02f * sr)) : 0.f;
            
            float totalDelayMod = wLfo + fLfo + azLfo + chewDelay + trackingDelay;
            float targetDelay = baseDelay + totalDelayMod;
            
            auto readHermite = [&](float delayAmt) {
                int delayInt = (int)std::floor(delayAmt);
                float frac = delayAmt - (float)delayInt;
                
                uint32_t i1 = writePos - delayInt;
                
                float y0 = delayBuffer[ch][(i1 + 1) & BUFFER_MASK]; 
                float y1 = delayBuffer[ch][i1 & BUFFER_MASK];       
                float y2 = delayBuffer[ch][(i1 - 1) & BUFFER_MASK]; 
                float y3 = delayBuffer[ch][(i1 - 2) & BUFFER_MASK]; 
                
                return interpolateHermite(y0, y1, y2, y3, frac);
            };

            float warped = readHermite(targetDelay);

            if (isDropouts && dDouble > 0.f) {
                float tap2Delay = targetDelay * 0.8f + (0.015f * sr);
                float tap2 = readHermite(tap2Delay);
                float blend = clamp(chewEnv + trackEnv, 0.f, 1.f) * (dDouble / 100.f);
                warped = warped * (1.f - blend) + tap2 * blend;
            }

            // Tape Hiss
            float white = random::normal(); 
            float noise = white;
            if (noiseType == 0) noise = brown[ch].process(white);
            else if (noiseType == 1) noise = pink[ch].process(white);
            
            float hissLevel = (nVol / 100.0f) * 0.1f;
            if (isVhs) {
                noiseVhsLp[ch].process(noise, 6000.f, sr);
                noise = noiseVhsLp[ch].lp * (1.f + hissMod * 0.4f);
                hissLevel *= 1.5f;
            }
            
            warped += noise * hissLevel;

            if (isVhs) {
                warped += humSignal * (nVol / 22.0f);
            }

            // Dropouts process ALL signal (Music + Hiss + Hum)
            if (isDropouts && dFiltVol > 0.f) {
                float filtVolAmt = dFiltVol / 100.f;
                muffleLp[ch].process(warped, muffleCutoff, sr);
                float muffled = muffleLp[ch].lp;
                warped = warped * (1.f - dropEnv * (filtVolAmt * 0.95f));
                warped = warped * (1.f - dropEnv) + muffled * dropEnv;
            }

            warped = std::tanh(warped);
            outputs[ch == 0 ? OUT_L_OUTPUT : OUT_R_OUTPUT].setVoltage(warped * 5.f);
        }

        writePos++;
    }
};

// ==========================================
// UI Definition & Text Label Widget
// ==========================================

struct TextLabel : widget::Widget {
    std::string text;
    TextLabel(math::Vec pos, std::string text) {
        this->box.pos = pos;
        this->text = text;
    }
    void draw(const DrawArgs& args) override {
        std::shared_ptr<window::Font> font = APP->window->uiFont;
        if (font) {
            nvgFillColor(args.vg, nvgRGBA(0, 0, 0, 255));
            nvgFontFaceId(args.vg, font->handle);
            nvgFontSize(args.vg, 12);
            nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_TOP);
            nvgText(args.vg, 0, 0, text.c_str(), NULL);
        }
    }
};

struct TapeSimWidget : ModuleWidget {
    TapeSimWidget(TapeSim* module) {
        setModule(module);
        
        setPanel(createPanel(asset::plugin(pluginInstance, "res/TAIP.svg")));
        
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        // Offset Y by -30 for plenty of clearance above knobs
        auto addKnobAndCv = [&](int paramId, int cvId, float x, float y, std::string name) {
            addChild(new TextLabel(Vec(x, y - 30), name)); 
            addParam(createParamCentered<RoundBlackKnob>(Vec(x, y+2), module, paramId));
            addInput(createInputCentered<PJ301MPort>(Vec(x, y + 35), module, cvId));
        };

        // Offset Y by -27 for clearance above switches
        addChild(new TextLabel(Vec(60, 18), "Tape Type"));
        addParam(createParamCentered<CKSS>(Vec(60, 45), module, TapeSim::TAPE_TYPE_PARAM));
        
        addChild(new TextLabel(Vec(225, 18), "Dropouts Enable"));
        addParam(createParamCentered<CKSS>(Vec(225, 45), module, TapeSim::DROPOUTS_EN_PARAM));
        
        addChild(new TextLabel(Vec(390, 18), "Noise Type"));
        addParam(createParamCentered<CKSSThree>(Vec(390, 45), module, TapeSim::NOISE_TYPE_PARAM));

        addKnobAndCv(TapeSim::SAT_AMT_PARAM, TapeSim::SAT_AMT_CV, 60, 95, "Sat Amt");
        addKnobAndCv(TapeSim::WOW_RATE_PARAM, TapeSim::WOW_RATE_CV, 170, 95, "Wow Rate");
        addKnobAndCv(TapeSim::WOW_DEPTH_PARAM, TapeSim::WOW_DEPTH_CV, 280, 95, "Wow Depth");
        addKnobAndCv(TapeSim::WOW_VAR_PARAM, TapeSim::WOW_VAR_CV, 390, 95, "Wow Var");

        addKnobAndCv(TapeSim::FLUTTER_RATE_PARAM, TapeSim::FLUTTER_RATE_CV, 60, 185, "Flut Rate");
        addKnobAndCv(TapeSim::FLUTTER_DEPTH_PARAM, TapeSim::FLUTTER_DEPTH_CV, 170, 185, "Flut Depth");
        addKnobAndCv(TapeSim::FLUTTER_VAR_PARAM, TapeSim::FLUTTER_VAR_CV, 280, 185, "Flut Var");
        addKnobAndCv(TapeSim::NOISE_VOL_PARAM, TapeSim::NOISE_VOL_CV, 390, 185, "Noise Vol");

        addKnobAndCv(TapeSim::DROP_FILTER_VOL_PARAM, TapeSim::DROP_FILTER_VOL_CV, 60, 275, "Drop Filt");
        addKnobAndCv(TapeSim::DROP_DOUBLING_PARAM, TapeSim::DROP_DOUBLING_CV, 170, 275, "Doubling");
        addKnobAndCv(TapeSim::DROP_DELAY_DRIFT_PARAM, TapeSim::DROP_DELAY_DRIFT_CV, 280, 275, "Drift");
        addKnobAndCv(TapeSim::DROP_PROB_PARAM, TapeSim::DROP_PROB_CV, 390, 275, "Drop Prob");

        // Offset Y for clearance above audio ports
        addChild(new TextLabel(Vec(40, 335), "IN L"));
        addInput(createInputCentered<PJ301MPort>(Vec(40, 360), module, TapeSim::IN_L_INPUT));
        
        addChild(new TextLabel(Vec(80, 335), "IN R"));
        addInput(createInputCentered<PJ301MPort>(Vec(80, 360), module, TapeSim::IN_R_INPUT));
        
        addChild(new TextLabel(Vec(370, 335), "OUT L"));
        addOutput(createOutputCentered<PJ301MPort>(Vec(370, 360), module, TapeSim::OUT_L_OUTPUT));
        
        addChild(new TextLabel(Vec(410, 335), "OUT R"));
        addOutput(createOutputCentered<PJ301MPort>(Vec(410, 360), module, TapeSim::OUT_R_OUTPUT));
    }
    void draw(const DrawArgs& args) override {
        ModuleWidget::draw(args);
        
        nvgFontSize(args.vg, 22);
        nvgFontFaceId(args.vg, APP->window->uiFont->handle);
        nvgFillColor(args.vg, nvgRGB(34, 34, 34));
        nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
        nvgText(args.vg, 227.0f, 361.0f, "TAIP", NULL);
    }
};

Model* modelTAIP = createModel<TapeSim, TapeSimWidget>("TAIP");