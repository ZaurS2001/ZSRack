#include "plugin.hpp"
#include "pffft.h"
#include <vector>
#include <cmath>
#include <algorithm>

struct AsyncBuffer {
    std::vector<float> data;
    int head = 0, tail = 0;
    int count = 0;
    int mask;

    AsyncBuffer(int size) {
        data.resize(size, 0.f);
        mask = size - 1;
    }

    void push(float v) {
        data[tail] = v;
        tail = (tail + 1) & mask;
        count++;
    }

    float pop() {
        if (count == 0) return 0.f;
        float v = data[head];
        head = (head + 1) & mask;
        count--;
        return v;
    }

    float peek(int offset) {
        return data[(head + offset) & mask];
    }

    void advance(int amount) {
        head = (head + amount) & mask;
        count -= amount;
    }

    void clear() { head = tail = count = 0; }
};

struct PAUL : Module {
    enum ParamId {
        PITCH_PARAM,
        STRETCH_PARAM,
        THRESHOLD_PARAM,
        RELEASE_PARAM,
        FILTER_PARAM,
        WINDOW_PARAM,
        WET_PARAM,
        RESET_PARAM,
        PARAMS_LEN
    };
    enum InputId {
        PITCH_CV,
        STRETCH_CV,
        THRESHOLD_CV,
        RELEASE_CV,
        FILTER_CV,
        WINDOW_CV,
        WET_CV,
        RESET_CV,
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

    // Internal State
    std::vector<float> stretchBufferL;
    std::vector<float> stretchBufferR;
    int writePtr = 0;
    int readPtr = 0;
    
    float peakL = 0.f, peakR = 0.f;
    bool isWriting = false;
    
    float ramp[32];
    int rampPtr = 0;
    
    float pWet = 0.5f;
    float pStretch = 4.f;
    float pThreshold = 0.0316f; // approx -30dB
    float pPitch = 0.f;
    float stretchCounter = 0.f;

    AsyncBuffer anaBufferL{262144};
    AsyncBuffer anaBufferR{262144};
    AsyncBuffer synBufferL{262144};
    AsyncBuffer synBufferR{262144};

    std::vector<float> prevWindowL;
    std::vector<float> prevWindowR;
    std::vector<float> paulWindow;
    std::vector<float> magBuffer; // Temp buffer for pitch shifting
    
    int sizes[9] = {256, 512, 1024, 2048, 4096, 8192, 16384, 32768, 65536};
    int currentWindowIdx = 4; // default 4096
    int windowSize = 4096;

    PFFFT_Setup* setups[9];
    float* pffftIn = nullptr;
    float* pffftOutL = nullptr;
    float* pffftOutR = nullptr;
    float* pffftWork = nullptr;

    // Filter properties (Dual LP & HP in series for smooth DJ crossfading)
    float pFilter_smooth = 0.f;
    float pCutoff_lp_smooth = 0.f;
    float pCutoff_hp_smooth = 0.f;
    
    float xh_left_lp = 0.f, xh_right_lp = 0.f;
    float xh_left_hp = 0.f, xh_right_hp = 0.f;

    float lastWetL = 0.f, lastWetR = 0.f; // Anti-jitter backup state

    dsp::SchmittTrigger resetTrigger;

    PAUL() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        
        configParam(PITCH_PARAM, -24.f, 24.f, 0.f, "Pitch", " st");
        configParam(STRETCH_PARAM, 1.f, 20.f, 4.f, "Stretch", "x");
        configParam(THRESHOLD_PARAM, -60.f, 0.f, -30.f, "Threshold", " dB");
        configParam(RELEASE_PARAM, 0.000025f, 0.01f, 0.005f, "Release", " s");
        configParam(FILTER_PARAM, -1.f, 1.f, 0.f, "Filter (LP <-> HP)");
        
        configParam(WINDOW_PARAM, 0.f, 8.f, 4.f, "Window Size");
        paramQuantities[WINDOW_PARAM]->snapEnabled = true;
        
        configParam(WET_PARAM, 0.f, 1.f, 1.f, "Dry/Wet Mix", "%", 0.f, 100.f);
        configParam(RESET_PARAM, 0.f, 1.f, 0.f, "Reset Buffer");

        configInput(PITCH_CV, "Pitch CV");
        configInput(STRETCH_CV, "Stretch CV");
        configInput(THRESHOLD_CV, "Threshold CV");
        configInput(RELEASE_CV, "Release CV");
        configInput(FILTER_CV, "Filter CV");
        configInput(WINDOW_CV, "Window Size CV");
        configInput(WET_CV, "Wet/Dry CV");
        configInput(RESET_CV, "Reset CV");
        
        configInput(IN_L, "Audio L");
        configInput(IN_R, "Audio R");
        
        configOutput(OUT_L, "Audio L");
        configOutput(OUT_R, "Audio R");

        for (int i = 0; i < 32; i++) {
            ramp[i] = (float)i / 31.f;
        }

        for (int i = 0; i < 9; i++) {
            setups[i] = pffft_new_setup(sizes[i], PFFFT_REAL);
        }

        pffftIn   = (float*)pffft_aligned_malloc(65536 * sizeof(float));
        pffftOutL = (float*)pffft_aligned_malloc(65536 * sizeof(float));
        pffftOutR = (float*)pffft_aligned_malloc(65536 * sizeof(float));
        pffftWork = (float*)pffft_aligned_malloc(65536 * sizeof(float));

        stretchBufferL.resize(1920000, 0.f);
        stretchBufferR.resize(1920000, 0.f);
        prevWindowL.resize(65536, 0.f);
        prevWindowR.resize(65536, 0.f);
        magBuffer.resize(65536, 0.f); // Extended to prevent out-of-bounds at Nyquist
        
        resetPaulWindow();
        resetBuffers();
    }

    ~PAUL() {
        if (pffftIn)   pffft_aligned_free(pffftIn);
        if (pffftOutL) pffft_aligned_free(pffftOutL);
        if (pffftOutR) pffft_aligned_free(pffftOutR);
        if (pffftWork) pffft_aligned_free(pffftWork);
        
        for (int i = 0; i < 9; i++) {
            if (setups[i]) pffft_destroy_setup(setups[i]);
        }
    }

    void resetPaulWindow() {
        paulWindow.resize(windowSize);
        for (int i = 0; i < windowSize; i++) {
            float x = -1.f + 2.f * i / (windowSize - 1.f);
            paulWindow[i] = std::pow(1.f - std::pow(x, 2.f), 1.25f);
        }
    }

    void resetBuffers() {
        anaBufferL.clear(); anaBufferR.clear();
        synBufferL.clear(); synBufferR.clear();
        std::fill(prevWindowL.begin(), prevWindowL.end(), 0.f);
        std::fill(prevWindowR.begin(), prevWindowR.end(), 0.f);
        lastWetL = lastWetR = 0.f;
    }

    void processFFT(AsyncBuffer& anaBuffer, float* out, int N, float pitchRate) {
        for (int i = 0; i < N; i++) {
            pffftIn[i] = anaBuffer.peek(i) * paulWindow[i];
        }

        PFFFT_Setup* setup = setups[currentWindowIdx];
        pffft_transform_ordered(setup, pffftIn, pffftWork, nullptr, PFFFT_FORWARD);

        // Store magnitudes cleanly (Fixing Nyquist overwrite bug)
        magBuffer[0] = std::abs(pffftWork[0]); // DC
        for (int k = 1; k < N / 2; k++) {
            float re = pffftWork[2 * k];
            float im = pffftWork[2 * k + 1];
            magBuffer[k] = std::sqrt(re * re + im * im);
        }
        magBuffer[N / 2] = std::abs(pffftWork[1]); // Nyquist correctly saved

        // Process Phase and Resample Magnitudes
        pffftWork[0] = magBuffer[0] * (random::uniform() > 0.5f ? 1.f : -1.f);
        pffftWork[1] = magBuffer[N / 2] * (random::uniform() > 0.5f ? 1.f : -1.f);

        for (int k = 1; k < N / 2; k++) {
            float srcIdx = k / pitchRate;
            float mag = 0.f;
            
            if (srcIdx < (N / 2)) {
                int iSrc = (int)srcIdx;
                float frac = srcIdx - iSrc;
                if (iSrc + 1 <= N / 2) {
                    mag = magBuffer[iSrc] * (1.f - frac) + magBuffer[iSrc + 1] * frac;
                } else {
                    mag = magBuffer[iSrc];
                }
            }

            float phase = random::uniform() * 2.f * M_PI;
            pffftWork[2 * k]     = mag * std::cos(phase);
            pffftWork[2 * k + 1] = mag * std::sin(phase);
        }

        pffft_transform_ordered(setup, pffftWork, out, nullptr, PFFFT_BACKWARD);

        float invN = 1.f / N;
        for (int i = 0; i < N; i++) {
            out[i] *= invN * paulWindow[i];
        }
    }

    void process(const ProcessArgs& args) override {
        // --- Reset trigger ---
        if (resetTrigger.process(params[RESET_PARAM].getValue() + inputs[RESET_CV].getVoltage())) {
            resetBuffers();
            std::fill(stretchBufferL.begin(), stretchBufferL.end(), 0.f);
            std::fill(stretchBufferR.begin(), stretchBufferR.end(), 0.f);
            writePtr = readPtr = rampPtr = 0;
            isWriting = false;
            peakL = peakR = 0.f;
            stretchCounter = 0.f;
            xh_left_lp = xh_right_lp = xh_left_hp = xh_right_hp = 0.f;
        }

        // --- Parameters Calculation ---
        float pitchTarget = clamp(params[PITCH_PARAM].getValue() + inputs[PITCH_CV].getVoltage() * 12.f, -24.f, 24.f);
        pPitch = pitchTarget - 0.9f * (pitchTarget - pPitch);
        float pitchRate = std::pow(2.f, pPitch / 12.f);

        float stretchTarget = clamp(params[STRETCH_PARAM].getValue() + inputs[STRETCH_CV].getVoltage() * 1.9f, 1.f, 20.f);
        pStretch = stretchTarget - 0.9f * (stretchTarget - pStretch);

        float threshDB = clamp(params[THRESHOLD_PARAM].getValue() + inputs[THRESHOLD_CV].getVoltage() * 6.f, -60.f, 0.f);
        pThreshold = std::pow(10.f, threshDB / 20.f);

        float rel = params[RELEASE_PARAM].getValue();
        if (inputs[RELEASE_CV].isConnected()) {
            float cv = inputs[RELEASE_CV].getVoltage() / 10.f;
            float minV = std::log(0.000025f);
            float maxV = std::log(0.01f);
            rel = std::exp(std::log(rel) + cv * (maxV - minV));
        }
        float tRelease = clamp(rel, 0.000025f, 0.01f);

        // DJ Filter Calculation
        float fTarget = clamp(params[FILTER_PARAM].getValue() + inputs[FILTER_CV].getVoltage() / 5.f, -1.f, 1.f);
        pFilter_smooth = 0.005f * fTarget + 0.995f * pFilter_smooth;

        float lp_F = std::min(0.f, pFilter_smooth);
        float cutoff_LP = clamp(20.f * std::pow(1000.f, 1.f + lp_F), 20.f, 20000.f);
        
        float hp_F = std::max(0.f, pFilter_smooth);
        float cutoff_HP = clamp(20.f * std::pow(1000.f, hp_F), 10.f, 20000.f);

        float omega_lp = clamp(2.f * cutoff_LP / args.sampleRate, 0.001f, 0.999f);
        float targetCutoff_LP = (std::tan(M_PI * omega_lp / 2.f) - 1.f) / (std::tan(M_PI * omega_lp / 2.f) + 1.f);

        float omega_hp = clamp(2.f * cutoff_HP / args.sampleRate, 0.001f, 0.999f);
        float targetCutoff_HP = (std::tan(M_PI * omega_hp / 2.f) - 1.f) / (std::tan(M_PI * omega_hp / 2.f) + 1.f);

        int winIdx = std::round(params[WINDOW_PARAM].getValue());
        if (inputs[WINDOW_CV].isConnected()) {
            winIdx += std::round(inputs[WINDOW_CV].getVoltage());
        }
        winIdx = clamp(winIdx, 0, 8);
        if (winIdx != currentWindowIdx) {
            currentWindowIdx = winIdx;
            windowSize = sizes[currentWindowIdx];
            resetPaulWindow();
            resetBuffers();
        }

        float tWet = clamp(params[WET_PARAM].getValue() + inputs[WET_CV].getVoltage() / 10.f, 0.f, 1.f);
        pWet = tWet - 0.95f * (tWet - pWet);

        // --- Audio Inputs ---
        float inL = inputs[IN_L].getVoltage() / 5.f;
        float inR = inputs[IN_R].isConnected() ? (inputs[IN_R].getVoltage() / 5.f) : inL;

        // --- Envelope Follower ---
        float inLevelL = std::abs(inL);
        float inLevelR = std::abs(inR);
        peakL = (inLevelL > peakL) ? inLevelL : ((1.f - tRelease) * peakL + tRelease * inLevelL);
        peakR = (inLevelR > peakR) ? inLevelR : ((1.f - tRelease) * peakR + tRelease * inLevelR);
        float maxPeak = std::max(peakL, peakR);

        // --- Writing to Stretch Buffer ---
        float threshOff = pThreshold * 0.7f;
        if (isWriting) {
            float rL = inL, rR = inR;
            if (rampPtr < 32) {
                rL *= ramp[rampPtr];
                rR *= ramp[rampPtr];
                rampPtr++;
            }
            stretchBufferL[writePtr] += rL;
            stretchBufferR[writePtr] += rR;

            if (maxPeak < threshOff) {
                isWriting = false;
            }
        } else {
            if (maxPeak >= pThreshold) {
                isWriting = true;
                writePtr = readPtr;
                rampPtr = 0;
                stretchBufferL[writePtr] += inL * ramp[rampPtr];
                stretchBufferR[writePtr] += inR * ramp[rampPtr];
                rampPtr++;
            }
        }

        writePtr++;
        if (writePtr >= 1920000) writePtr = 0;

        // --- Stretch Timing Accumulator ---
        int numIterations = std::floor(stretchCounter);
        for (int j = 0; j < numIterations; j++) {
            anaBufferL.push(stretchBufferL[readPtr]);
            anaBufferR.push(stretchBufferR[readPtr]);
            
            stretchBufferL[readPtr] = 0.f;
            stretchBufferR[readPtr] = 0.f;
            readPtr++;
            if (readPtr >= 1920000) readPtr = 0;
            stretchCounter -= 1.f;
        }
        stretchCounter += 1.f / pStretch;

        // --- FFT Processing Block ---
        int halfWindowSize = windowSize / 2;
        int hopSize = std::floor((float)halfWindowSize / pStretch);

        while (anaBufferL.count >= windowSize) {
            processFFT(anaBufferL, pffftOutL, windowSize, pitchRate);
            processFFT(anaBufferR, pffftOutR, windowSize, pitchRate);

            // Overlap Add
            for (int k = 0; k < halfWindowSize; k++) {
                float sL = pffftOutL[k] + prevWindowL[k];
                float sR = pffftOutR[k] + prevWindowR[k];
                synBufferL.push(sL);
                synBufferR.push(sR);

                prevWindowL[k] = pffftOutL[k + halfWindowSize];
                prevWindowR[k] = pffftOutR[k + halfWindowSize];
            }

            anaBufferL.advance(hopSize);
            anaBufferR.advance(hopSize);
        }

	// --- Continuous Mix and Output ---
        float wetL = 0.f, wetR = 0.f;
        if (synBufferL.count >= 1) {
            wetL = synBufferL.pop();
            wetR = synBufferR.pop();
            lastWetL = wetL; // Store for starvation prevention
            lastWetR = wetR;
        } else {
            wetL = lastWetL;
            wetR = lastWetR;
        }

        // 1. Apply Filter ONLY to the Wet Signal
        pCutoff_lp_smooth = 0.001f * targetCutoff_LP + 0.999f * pCutoff_lp_smooth;
        pCutoff_hp_smooth = 0.001f * targetCutoff_HP + 0.999f * pCutoff_hp_smooth;

        // Left Channel Cascade
        float xhl_new_lp = wetL - pCutoff_lp_smooth * xh_left_lp;
        float ap_l_lp = pCutoff_lp_smooth * xhl_new_lp + xh_left_lp;
        xh_left_lp = xhl_new_lp;
        float wetL_lp = 0.5f * (wetL + ap_l_lp);

        float xhl_new_hp = wetL_lp - pCutoff_hp_smooth * xh_left_hp;
        float ap_l_hp = pCutoff_hp_smooth * xhl_new_hp + xh_left_hp;
        xh_left_hp = xhl_new_hp;
        float filteredWetL = 0.5f * (wetL_lp - ap_l_hp);

        // Right Channel Cascade
        float xhr_new_lp = wetR - pCutoff_lp_smooth * xh_right_lp;
        float ap_r_lp = pCutoff_lp_smooth * xhr_new_lp + xh_right_lp;
        xh_right_lp = xhr_new_lp;
        float wetR_lp = 0.5f * (wetR + ap_r_lp);

        float xhr_new_hp = wetR_lp - pCutoff_hp_smooth * xh_right_hp;
        float ap_r_hp = pCutoff_hp_smooth * xhr_new_hp + xh_right_hp;
        xh_right_hp = xhr_new_hp;
        float filteredWetR = 0.5f * (wetR_lp - ap_r_hp);

        // Center Bypass safety 
        if (std::abs(pFilter_smooth) < 0.005f) {
            filteredWetL = wetL;
            filteredWetR = wetR;
            // Decay states gently to prevent pops when turning the knob back
            xh_left_lp *= 0.99f; xh_right_lp *= 0.99f;
            xh_left_hp *= 0.99f; xh_right_hp *= 0.99f;
        }

        // 2. NOW blend the filtered wet signal with the untouched dry signal
        float outL = filteredWetL * pWet + inL * (1.f - pWet);
        float outR = filteredWetR * pWet + inR * (1.f - pWet);

        outputs[OUT_L].setVoltage(clamp(outL, -1.f, 1.f) * 5.f);
        outputs[OUT_R].setVoltage(clamp(outR, -1.f, 1.f) * 5.f);
    }
};

struct PAULWidget : ModuleWidget {
    PAULWidget(PAUL* module) {
        setModule(module);
        setPanel(createPanel(asset::plugin(pluginInstance, "res/PAUL.svg")));

        float cx = 35.f;
        float cvx = 12.f;
        
        float pyPitch     = 16.f;
        float pyStretch   = 30.f;
        float pyThreshold = 44.f;
        float pyRelease   = 58.f;
        float pyFilter    = 72.f;
        float pyWindow    = 86.f;
        float pyWet       = 100.f;
        float pyReset     = 112.f;
        float pyJacks     = 123.f;

        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(cvx, pyPitch)), module, PAUL::PITCH_CV));
        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(cx, pyPitch)), module, PAUL::PITCH_PARAM));

        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(cvx, pyStretch)), module, PAUL::STRETCH_CV));
        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(cx, pyStretch)), module, PAUL::STRETCH_PARAM));

        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(cvx, pyThreshold)), module, PAUL::THRESHOLD_CV));
        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(cx, pyThreshold)), module, PAUL::THRESHOLD_PARAM));

        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(cvx, pyRelease)), module, PAUL::RELEASE_CV));
        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(cx, pyRelease)), module, PAUL::RELEASE_PARAM));

        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(cvx, pyFilter)), module, PAUL::FILTER_CV));
        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(cx, pyFilter)), module, PAUL::FILTER_PARAM));

        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(cvx, pyWindow)), module, PAUL::WINDOW_CV));
        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(cx, pyWindow)), module, PAUL::WINDOW_PARAM));

        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(cvx, pyWet)), module, PAUL::WET_CV));
        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(cx, pyWet)), module, PAUL::WET_PARAM));

        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(cvx, pyReset)), module, PAUL::RESET_CV));
        addParam(createParamCentered<TL1105>(mm2px(Vec(cx, pyReset)), module, PAUL::RESET_PARAM));

        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(8.f, pyJacks)), module, PAUL::IN_L));
        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(18.f, pyJacks)), module, PAUL::IN_R));
        addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(32.8f, pyJacks)), module, PAUL::OUT_L));
        addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(42.8f, pyJacks)), module, PAUL::OUT_R));

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        //addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        //addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
    }

    void draw(const DrawArgs& args) override {
        nvgBeginPath(args.vg);
        nvgRect(args.vg, 0, 0, box.size.x, box.size.y);
        nvgFillColor(args.vg, nvgRGB(0xe9, 0xe9, 0xe9));
        nvgFill(args.vg);

        ModuleWidget::draw(args);
        
        std::shared_ptr<Font> font = APP->window->uiFont;
        if (!font) return;

        nvgFontFaceId(args.vg, font->handle);
        nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
        nvgFillColor(args.vg, nvgRGB(0, 0, 0));

        nvgFontSize(args.vg, mm2px(5.5f));
        nvgText(args.vg, mm2px(25.4f), mm2px(5.f), "PAUL", NULL);

        nvgFontSize(args.vg, mm2px(3.2f));
        nvgText(args.vg, mm2px(12.f), mm2px(10.f), "CV", NULL);
        nvgText(args.vg, mm2px(35.f), mm2px(10.f), "Pitch", NULL);
        nvgText(args.vg, mm2px(35.f), mm2px(23.f), "Stretch", NULL);
        nvgText(args.vg, mm2px(35.f), mm2px(37.f), "Threshold", NULL);
        nvgText(args.vg, mm2px(35.f), mm2px(51.f), "Release", NULL);
        nvgText(args.vg, mm2px(35.f), mm2px(65.f), "Filter", NULL);
        nvgText(args.vg, mm2px(35.f), mm2px(79.f), "Window", NULL);
        nvgText(args.vg, mm2px(35.f), mm2px(93.f), "Dry/Wet", NULL);
        nvgText(args.vg, mm2px(35.f), mm2px(107.f), "Reset", NULL);

        nvgFontSize(args.vg, mm2px(2.8f));
        nvgText(args.vg, mm2px(8.f), mm2px(118.f), "IN L", NULL);
        nvgText(args.vg, mm2px(18.f), mm2px(118.f), "IN R", NULL);
        nvgText(args.vg, mm2px(32.8f), mm2px(118.f), "OUT L", NULL);
        nvgText(args.vg, mm2px(42.8f), mm2px(118.f), "OUT R", NULL);
    }
};

Model* modelPAUL = createModel<PAUL, PAULWidget>("PAUL");