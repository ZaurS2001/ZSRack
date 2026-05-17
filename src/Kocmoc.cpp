#include "plugin.hpp"
#include <vector>
#include <cmath>
#include <string>

// Reverb & Delay building blocks
struct Comb {
    std::vector<float> buf;
    int idx = 0;
    float dampState = 0.f;
    Comb(int size) { buf.resize(size, 0.f); }
    float process(float in, float fb, float damp) {
        float out = buf[idx];
        dampState = out * (1.f - damp) + dampState * damp;
        buf[idx] = in + dampState * fb;
        idx = (idx + 1) % buf.size();
        return out;
    }
};

struct Allpass {
    std::vector<float> buf;
    int idx = 0;
    Allpass(int size) { buf.resize(size, 0.f); }
    float process(float in) {
        float delayed = buf[idx];
        float out = -in + delayed;
        buf[idx] = in + delayed * 0.5f;
        idx = (idx + 1) % buf.size();
        return out;
    }
};

struct ADSR {
    float val = 0.f;
    int state = 0; // 0=idle, 1=A, 2=D, 3=S, 4=R
    
    void step(bool gate, float a, float d, float s, float r, float dt) {
        float aT = 0.001f + a * 5.f; 
        float dT = 0.001f + d * 5.f; 
        float rT = 0.001f + r * 5.f;
        
        if (gate) {
            if (state == 0 || state == 4) state = 1; // trigger
        } else {
            if (state == 1 || state == 2 || state == 3) state = 4; // release
        }
        
        if (state == 1) { 
            val += dt / aT; 
            if (val >= 1.f) { val = 1.f; state = 2; } 
        } else if (state == 2) { 
            val -= dt / dT; 
            if (val <= s) { val = s; state = 3; } 
        } else if (state == 3) { 
            val = s; 
        } else if (state == 4) { 
            val -= dt / rT; 
            if (val <= 0.f) { val = 0.f; state = 0; } 
        }
    }
};

struct PulseGen {
    float time = 0.f;
    void trigger(float duration) { time = duration; }
    bool process(float dt) {
        if (time > 0.f) { time -= dt; return true; }
        return false;
    }
};

struct Kocmoc : Module {
    enum ParamId {
        W1_SAW, W1_PLS, W1_SIN, W1_NSE, PW1, CRS1, DET1, LVL1,
        A1, D1, S1, R1,
        W2_SAW, W2_PLS, W2_SIN, W2_NSE, PW2, CRS2, DET2, LVL2,
        A2, D2, S2, R2, AMT,
        CUTOFF, RESO,
        PLAY, REVERSE, TEMPO,
        D_MIX, D_TIME, D_FB, R_MIX, R_SIZE, R_DAMP,
        SEQ_NOTE_0, SEQ_ACTIVE_0 = SEQ_NOTE_0 + 16,
        PARAMS_LEN = SEQ_ACTIVE_0 + 16
    };
    enum InputId {
        VOCT_IN, CLK_IN,
        PW1_CV, CRS1_CV, DET1_CV, LVL1_CV, ENV1_CV,
        PW2_CV, CRS2_CV, DET2_CV, LVL2_CV, ENV2_CV,
        DMIX_CV, DTIME_CV, DFB_CV, RMIX_CV, RSIZE_CV, RDAMP_CV,
        INPUTS_LEN
    };
    enum OutputId { 
        OUT_L, OUT_R, 
        VOCT_OUT, CLK_OUT,
        OUTPUTS_LEN 
    };

    float phase1 = 0.f, phase2 = 0.f;
    float bpmPhase = 0.f;
    int currentStep = 0;
    int lastActiveStep = 0; // Remembers the pitch to hold through rests
    bool wasClkHigh = false;

    ADSR env1, env2;
    PulseGen clockPulse;
    
    // Andy Simper SVF states
    float ic1eq = 0.f;
    float ic2eq = 0.f; 

    std::vector<float> delayBuf;
    int delayIdx = 0;

    Comb c1{1116}, c2{1188}, c3{1277}, c4{1356};
    Allpass a1{225}, a2{341};

    Kocmoc() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN);
        delayBuf.resize(192000 * 22, 0.f); // Pre-allocate enough for 22s at 192kHz

        // Param Configurations
        configParam(W1_SAW, 0.f, 1.f, 1.f, "Osc 1 Saw");
        configParam(W1_PLS, 0.f, 1.f, 0.f, "Osc 1 Pulse");
        configParam(W1_SIN, 0.f, 1.f, 0.f, "Osc 1 Sin");
        configParam(W1_NSE, 0.f, 1.f, 0.f, "Osc 1 Noise");
        configParam(PW1, 0.f, 1.f, 0.5f, "Osc 1 Pulsewidth");
        configParam(CRS1, -24.f, 24.f, 0.f, "Osc 1 Coarse Tune");
        configParam(DET1, -1.f, 1.f, 0.f, "Osc 1 Detune");
        configParam(LVL1, 0.f, 1.f, 0.8f, "Osc 1 Level");

        configParam(W2_SAW, 0.f, 1.f, 1.f, "Osc 2 Saw");
        configParam(W2_PLS, 0.f, 1.f, 0.f, "Osc 2 Pulse");
        configParam(W2_SIN, 0.f, 1.f, 0.f, "Osc 2 Sin");
        configParam(W2_NSE, 0.f, 1.f, 0.f, "Osc 2 Noise");
        configParam(PW2, 0.f, 1.f, 0.5f, "Osc 2 Pulsewidth");
        configParam(CRS2, -24.f, 24.f, 0.f, "Osc 2 Coarse Tune");
        configParam(DET2, -1.f, 1.f, 0.f, "Osc 2 Detune");
        configParam(LVL2, 0.f, 1.f, 0.8f, "Osc 2 Level");

        configParam(A1, 0.f, 1.f, 0.1f, "ADSR 1 Attack");
        configParam(D1, 0.f, 1.f, 0.3f, "ADSR 1 Decay");
        configParam(S1, 0.f, 1.f, 0.5f, "ADSR 1 Sustain");
        configParam(R1, 0.f, 1.f, 0.2f, "ADSR 1 Release");

        configParam(A2, 0.f, 1.f, 0.1f, "ADSR 2 Attack");
        configParam(D2, 0.f, 1.f, 0.3f, "ADSR 2 Decay");
        configParam(S2, 0.f, 1.f, 0.5f, "ADSR 2 Sustain");
        configParam(R2, 0.f, 1.f, 0.2f, "ADSR 2 Release");
        configParam(AMT, 0.f, 1.f, 0.5f, "Env Filter Amount");

        configParam(CUTOFF, 0.f, 22000.f, 5000.f, "Filter Cutoff", " Hz");
        configParam(RESO, 0.f, 1.f, 0.1f, "Filter Resonance");

        configParam(PLAY, 0.f, 1.f, 0.f, "Play Sequence");
        configParam(REVERSE, 0.f, 1.f, 0.f, "Reverse Sequence");
        configParam(TEMPO, 30.f, 300.f, 120.f, "Tempo", " BPM");
        for (int i = 0; i < 16; i++) {
            configParam(SEQ_NOTE_0 + i, 0.f, 127.f, 69.f, "Step Note");
            configParam(SEQ_ACTIVE_0 + i, 0.f, 1.f, 1.f, "Step Active");
        }

        configParam(D_MIX, 0.f, 1.f, 0.2f, "Delay Mix");
        configParam(D_TIME, 0.f, 22.f, 0.5f, "Delay Time", " s");
        configParam(D_FB, -1.f, 1.f, 0.3f, "Decay Time (Feedback)");
        configParam(R_MIX, 0.f, 1.f, 0.2f, "Reverb Mix");
        configParam(R_SIZE, -1.f, 1.f, 0.f, "Room Size");
        configParam(R_DAMP, -1.f, 1.f, 0.f, "Damp");

        // Input & Output Port Tooltips
        configInput(VOCT_IN, "External V/OCT");
        configInput(CLK_IN, "External Clock");
        
        configInput(PW1_CV, "Osc 1 Pulsewidth CV");
        configInput(CRS1_CV, "Osc 1 Coarse Tune CV");
        configInput(DET1_CV, "Osc 1 Detune CV");
        configInput(LVL1_CV, "Osc 1 Level CV");
        configInput(ENV1_CV, "ADSR 1 Gate CV");
        
        configInput(PW2_CV, "Osc 2 Pulsewidth CV");
        configInput(CRS2_CV, "Osc 2 Coarse Tune CV");
        configInput(DET2_CV, "Osc 2 Detune CV");
        configInput(LVL2_CV, "Osc 2 Level CV");
        configInput(ENV2_CV, "ADSR 2 Gate CV");
        
        configInput(DMIX_CV, "Delay Mix CV");
        configInput(DTIME_CV, "Delay Time CV");
        configInput(DFB_CV, "Delay Feedback CV");
        configInput(RMIX_CV, "Reverb Mix CV");
        configInput(RSIZE_CV, "Reverb Size CV");
        configInput(RDAMP_CV, "Reverb Damp CV");

        configOutput(OUT_L, "Audio Out (Left)");
        configOutput(OUT_R, "Audio Out (Right)");
        configOutput(VOCT_OUT, "Sequencer V/OCT Out");
        configOutput(CLK_OUT, "Sequencer Clock Out");
    }

    float getOsc(float phase, float pw, float pSaw, float pPls, float pSin, float pNse) {
        float out = 0.f;
        if (pSaw > 0.5f) out = 2.f * phase - 1.f;
        else if (pPls > 0.5f) {
            float sq = (phase < pw) ? 1.f : -1.f;
            float tri = 1.f - 4.f * std::abs(std::fmod(phase * 2.f + 0.5f, 1.f) - 0.5f);
            float saw = 2.f * std::fmod(phase * 4.f, 1.f) - 1.f;
            out = sq * (tri + saw * (phase > 0.8f ? 1.f : 0.f)) * 0.5f;
        }
        else if (pSin > 0.5f) out = std::sin(2.f * M_PI * phase);
        else if (pNse > 0.5f) out = (float)rand() / RAND_MAX * 2.f - 1.f;
        else out = 2.f * phase - 1.f; // Fallback
        return out;
    }

    void process(const ProcessArgs& args) override {
        bool isPlaying = params[PLAY].getValue() > 0.5f;
        bool stepTriggered = false;
        bool rawClockGate = false;

        // Sequencer Clock Logic
        if (isPlaying) {
            if (inputs[CLK_IN].isConnected()) {
                bool clkHigh = inputs[CLK_IN].getVoltage() > 1.f;
                if (clkHigh && !wasClkHigh) {
                    stepTriggered = true;
                }
                rawClockGate = clkHigh;
                wasClkHigh = clkHigh;
            } else {
                float tempo = params[TEMPO].getValue();
                bpmPhase += (tempo / 60.f) * args.sampleTime * 4.f; // 16ths
                if (bpmPhase >= 1.f) {
                    bpmPhase -= 1.f;
                    stepTriggered = true;
                }
                rawClockGate = bpmPhase < 0.5f;
            }

            if (stepTriggered) {
                int dir = params[REVERSE].getValue() > 0.5f ? -1 : 1;
                currentStep = (currentStep + dir + 16) % 16;
                // Only update the active pitch if the step is actually active!
                if (params[SEQ_ACTIVE_0 + currentStep].getValue() > 0.5f) {
                    lastActiveStep = currentStep;
                }
                clockPulse.trigger(0.01f); // Trigger 10ms clock pulse out
            }
        } else {
            currentStep = 0;
            bpmPhase = 0.f;
            wasClkHigh = false;
        }

        bool seqGate = false;
        if (isPlaying && params[SEQ_ACTIVE_0 + currentStep].getValue() > 0.5f) {
            seqGate = rawClockGate;
        }

        // Gates logic (Combines Sequencer + CV Ins)
        bool env1_gate = seqGate;
        if (inputs[ENV1_CV].isConnected() && inputs[ENV1_CV].getVoltage() >= 1.f) env1_gate = true;

        bool env2_gate = seqGate;
        if (inputs[ENV2_CV].isConnected() && inputs[ENV2_CV].getVoltage() >= 1.f) env2_gate = true;

        env1.step(env1_gate, params[A1].getValue(), params[D1].getValue(), params[S1].getValue(), params[R1].getValue(), args.sampleTime);
        env2.step(env2_gate, params[A2].getValue(), params[D2].getValue(), params[S2].getValue(), params[R2].getValue(), args.sampleTime);

        float e1 = env1.val;
        float e2 = env2.val;

        // Note Calculation - always holds pitch of the last active step during rests
        float seqPitch = (params[SEQ_NOTE_0 + lastActiveStep].getValue() - 60.f) / 12.f;
        float basePitch = inputs[VOCT_IN].getVoltage() + seqPitch;
        
        // Output voltages for sequencer
        outputs[VOCT_OUT].setVoltage(seqPitch);
        outputs[CLK_OUT].setVoltage(clockPulse.process(args.sampleTime) ? 10.f : 0.f);

        // Osc 1
        float p1 = basePitch + (params[CRS1].getValue() + inputs[CRS1_CV].getVoltage()) / 12.f + (params[DET1].getValue() + inputs[DET1_CV].getVoltage() / 5.f) / 100.f;
        float freq1 = 261.6256f * std::pow(2.f, p1);
        phase1 += freq1 * args.sampleTime;
        if (phase1 >= 1.f) phase1 -= 1.f;
        float pw1 = clamp(params[PW1].getValue() + inputs[PW1_CV].getVoltage() / 10.f, 0.01f, 0.99f);
        float osc1 = getOsc(phase1, pw1, params[W1_SAW].getValue(), params[W1_PLS].getValue(), params[W1_SIN].getValue(), params[W1_NSE].getValue());

        // Osc 2
        float p2 = basePitch + (params[CRS2].getValue() + inputs[CRS2_CV].getVoltage()) / 12.f + (params[DET2].getValue() + inputs[DET2_CV].getVoltage() / 5.f) / 100.f;
        float freq2 = 261.6256f * std::pow(2.f, p2);
        phase2 += freq2 * args.sampleTime;
        if (phase2 >= 1.f) phase2 -= 1.f;
        float pw2 = clamp(params[PW2].getValue() + inputs[PW2_CV].getVoltage() / 10.f, 0.01f, 0.99f);
        float osc2 = getOsc(phase2, pw2, params[W2_SAW].getValue(), params[W2_PLS].getValue(), params[W2_SIN].getValue(), params[W2_NSE].getValue());

        float lvl1 = clamp(params[LVL1].getValue() + inputs[LVL1_CV].getVoltage() / 10.f, 0.f, 1.f);
        float lvl2 = clamp(params[LVL2].getValue() + inputs[LVL2_CV].getVoltage() / 10.f, 0.f, 1.f);
        float mix = (osc1 * e1 * lvl1) + (osc2 * e2 * lvl2);

        // SVF Filter (Andy Simper topology - guaranteed stable)
        float cutCV = ((e1 + e2) * 0.5f) * params[AMT].getValue() * 10000.f;
        float cutoff = clamp(params[CUTOFF].getValue() + cutCV, 10.f, args.sampleRate / 3.f);
        float reso = clamp(params[RESO].getValue(), 0.f, 0.99f);

        float g = std::tan(M_PI * cutoff * args.sampleTime);
        float R = 1.0f - reso; 
        if (R < 0.01f) R = 0.01f;
        
        float f_a1 = 1.0f / (1.0f + g * (g + 2.0f * R));
        float f_a2 = g * f_a1;
        float f_a3 = g * f_a2;
        
        float v3 = mix - ic2eq;
        float v1 = f_a1 * ic1eq + f_a2 * v3;
        float v2 = ic2eq + f_a2 * ic1eq + f_a3 * v3;
        
        ic1eq = 2.0f * v1 - ic1eq;
        ic2eq = 2.0f * v2 - ic2eq;
        
        float filterOut = v2; // Lowpass output

        // Delay
        float dTime = clamp(params[D_TIME].getValue() + inputs[DTIME_CV].getVoltage() * 2.2f, 0.001f, 22.f);
        int dSamps = clamp((int)(dTime * args.sampleRate), 1, (int)delayBuf.size() - 1);
        
        int rIdx = delayIdx - dSamps;
        if (rIdx < 0) rIdx += delayBuf.size();
        
        float dOut = delayBuf[rIdx];
        float dFb = clamp(params[D_FB].getValue() + inputs[DFB_CV].getVoltage() / 5.f, -0.99f, 0.99f);
        
        delayBuf[delayIdx] = filterOut + dOut * dFb;
        delayIdx++;
        if (delayIdx >= (int)delayBuf.size()) delayIdx = 0;
        
        float dMixAmt = clamp(params[D_MIX].getValue() + inputs[DMIX_CV].getVoltage() / 10.f, 0.f, 1.f);
        float wetDly = filterOut * (1.f - dMixAmt) + dOut * dMixAmt;

        // Reverb
        float rSize = clamp(params[R_SIZE].getValue() + inputs[RSIZE_CV].getVoltage() / 5.f, -1.f, 1.f);
        float fb = 0.84f + rSize * 0.14f;
        float rDmp = clamp(params[R_DAMP].getValue() + inputs[RDAMP_CV].getVoltage() / 5.f, -1.f, 1.f);
        float damp = (rDmp + 1.f) * 0.45f;
        
        float cOut = c1.process(wetDly, fb, damp) + c2.process(wetDly, fb, damp) + c3.process(wetDly, fb, damp) + c4.process(wetDly, fb, damp);
        float apOut = a2.process(a1.process(cOut));

        float rMixAmt = clamp(params[R_MIX].getValue() + inputs[RMIX_CV].getVoltage() / 10.f, 0.f, 1.f);
        float finalOut = wetDly * (1.f - rMixAmt) + apOut * rMixAmt;

        outputs[OUT_L].setVoltage(finalOut * 5.f);
        outputs[OUT_R].setVoltage(finalOut * 5.f);
    }
};

struct RectSlider : Knob {
    std::string label;
    void draw(const DrawArgs& args) override {
        ParamQuantity* pq = getParamQuantity();
        float val = pq ? (pq->getValue() - pq->getMinValue()) / (pq->getMaxValue() - pq->getMinValue()) : 0.f;
        float fillY = box.size.y * val;
        
        nvgBeginPath(args.vg);
        nvgRect(args.vg, 0, box.size.y - fillY, box.size.x, fillY);
        nvgFillColor(args.vg, nvgRGB(255, 255, 255));
        nvgFill(args.vg);

        if (APP->window->uiFont) {
            nvgFontFaceId(args.vg, APP->window->uiFont->handle);
            nvgFontSize(args.vg, 12);
            nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
            nvgFillColor(args.vg, val > 0.5f ? nvgRGB(0,0,0) : nvgRGB(255,255,255));
            nvgText(args.vg, box.size.x/2.f, box.size.y/2.f, label.c_str(), NULL);
        }
    }
};

struct ArcSlider : Knob {
    std::string label;
    void draw(const DrawArgs& args) override {
        ParamQuantity* pq = getParamQuantity();
        float val = pq ? (pq->getValue() - pq->getMinValue()) / (pq->getMaxValue() - pq->getMinValue()) : 0.f;
        float cx = box.size.x / 2.f, cy = box.size.y / 2.f;
        float r = std::min(cx, cy) * 0.7f;
        
        nvgBeginPath(args.vg);
        nvgArc(args.vg, cx, cy, r, NVG_PI * 0.75f, NVG_PI * 2.25f, NVG_CW);
        nvgStrokeColor(args.vg, nvgRGBA(150, 150, 150, 100));
        nvgStrokeWidth(args.vg, 8.f);
        nvgStroke(args.vg);
        
        nvgBeginPath(args.vg);
        nvgArc(args.vg, cx, cy, r, NVG_PI * 0.75f, NVG_PI * 0.75f + val * (NVG_PI * 1.5f), NVG_CW);
        nvgStrokeColor(args.vg, nvgRGB(255, 255, 255));
        nvgStrokeWidth(args.vg, 8.f);
        nvgStroke(args.vg);

        if (APP->window->uiFont) {
            nvgFontFaceId(args.vg, APP->window->uiFont->handle);
            nvgFontSize(args.vg, 14);
            nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
            nvgFillColor(args.vg, nvgRGB(255,255,255));
            nvgText(args.vg, cx, cy, label.c_str(), NULL);
        }
    }
};

struct ToggleBtn : ParamWidget {
    std::string label;
    void onButton(const ButtonEvent& e) override {
        ParamQuantity* pq = getParamQuantity();
        if (e.action == GLFW_PRESS && e.button == GLFW_MOUSE_BUTTON_LEFT && pq) {
            pq->setValue(pq->getValue() > 0.5f ? 0.f : 1.f);
            e.consume(this);
        }
    }
    void draw(const DrawArgs& args) override {
        ParamQuantity* pq = getParamQuantity();
        bool active = pq && pq->getValue() > 0.5f;

        nvgBeginPath(args.vg);
        nvgRect(args.vg, 0, 0, box.size.x, box.size.y);
        nvgFillColor(args.vg, active ? nvgRGB(255, 255, 255) : nvgRGB(0, 0, 0));
        nvgFill(args.vg);

        if (APP->window->uiFont) {
            nvgFontFaceId(args.vg, APP->window->uiFont->handle);
            nvgFontSize(args.vg, 12);
            nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
            nvgFillColor(args.vg, active ? nvgRGB(0,0,0) : nvgRGB(255,255,255));
            nvgText(args.vg, box.size.x/2.f, box.size.y/2.f, label.c_str(), NULL);
        }
    }
};

struct WaveBtn : ParamWidget {
    std::string label;
    std::vector<int> groupIds;
    void onButton(const ButtonEvent& e) override {
        if (e.action == GLFW_PRESS && e.button == GLFW_MOUSE_BUTTON_LEFT) {
            ParamQuantity* pq = getParamQuantity();
            if (pq) {
                pq->setValue(1.f);
                for (int id : groupIds) {
                    if (id != pq->paramId && module) {
                        module->paramQuantities[id]->setValue(0.f);
                    }
                }
            }
            e.consume(this);
        }
    }
    void draw(const DrawArgs& args) override {
        ParamQuantity* pq = getParamQuantity();
        bool active = pq && pq->getValue() > 0.5f;

        nvgBeginPath(args.vg);
        nvgRect(args.vg, 0, 0, box.size.x, box.size.y);
        nvgFillColor(args.vg, active ? nvgRGB(255, 255, 255) : nvgRGB(0, 0, 0));
        nvgFill(args.vg);

        if (APP->window->uiFont) {
            nvgFontFaceId(args.vg, APP->window->uiFont->handle);
            nvgFontSize(args.vg, 12);
            nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
            nvgFillColor(args.vg, active ? nvgRGB(0,0,0) : nvgRGB(255,255,255));
            nvgText(args.vg, box.size.x/2.f, box.size.y/2.f, label.c_str(), NULL);
        }
    }
};

struct SeqCell : ParamWidget {
    int stepIndex = 0;
    bool isDragging = false;
    float dragAccum = 0.f;

    void onButton(const ButtonEvent& e) override {
        if (e.action == GLFW_PRESS && e.button == GLFW_MOUSE_BUTTON_LEFT) {
            isDragging = false;
            dragAccum = 0.f;
            e.consume(this); 
        }
        if (e.action == GLFW_RELEASE && e.button == GLFW_MOUSE_BUTTON_LEFT) {
            if (!isDragging && module) {
                ParamQuantity* actParam = module->paramQuantities[Kocmoc::SEQ_ACTIVE_0 + stepIndex];
                if (actParam) actParam->setValue(actParam->getValue() > 0.5f ? 0.f : 1.f);
            }
        }
    }
    void onDragStart(const DragStartEvent& e) override {}
    void onDragMove(const DragMoveEvent& e) override {
        dragAccum += std::abs(e.mouseDelta.y) + std::abs(e.mouseDelta.x);
        if (dragAccum > 2.0f) isDragging = true;
        
        if (isDragging) {
            ParamQuantity* pq = getParamQuantity();
            if (pq) {
                float delta = -e.mouseDelta.y;
                pq->setValue(pq->getValue() + delta * 0.5f);
            }
        }
    }
    void draw(const DrawArgs& args) override {
        bool active = false;
        if (module) {
            ParamQuantity* actParam = module->paramQuantities[Kocmoc::SEQ_ACTIVE_0 + stepIndex];
            if (actParam) active = actParam->getValue() > 0.5f;
        }

        nvgBeginPath(args.vg);
        nvgRect(args.vg, 0, 0, box.size.x, box.size.y);
        nvgFillColor(args.vg, active ? nvgRGB(255, 255, 255) : nvgRGB(0, 0, 0));
        nvgFill(args.vg);

        ParamQuantity* pq = getParamQuantity();
        if (APP->window->uiFont && pq) {
            std::string txt = std::to_string((int)std::round(pq->getValue()));
            nvgFontFaceId(args.vg, APP->window->uiFont->handle);
            nvgFontSize(args.vg, 14);
            nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
            nvgFillColor(args.vg, active ? nvgRGB(0,0,0) : nvgRGB(255,255,255));
            nvgText(args.vg, box.size.x/2.f, box.size.y/2.f, txt.c_str(), NULL);
        }
    }
};

struct LabelWidget : Widget {
    std::string text; int size;
    LabelWidget(Vec pos, std::string text, int size = 10) : text(text), size(size) { box.pos = pos; }
    void draw(const DrawArgs& args) override {
        if (APP->window->uiFont) {
            nvgFontFaceId(args.vg, APP->window->uiFont->handle);
            nvgFontSize(args.vg, size);
            nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
            nvgFillColor(args.vg, nvgRGB(255, 255, 255));
            nvgText(args.vg, 0, 0, text.c_str(), NULL);
        }
    }
};

struct KocmocWidget : ModuleWidget {
    KocmocWidget(Kocmoc* module) {
        setModule(module);
        setPanel(createPanel(asset::plugin(pluginInstance, "res/Kocmoc.svg")));
        auto addRect = [&](int id, float x, float y, float w, float h, std::string lbl) {
            RectSlider* s = createParam<RectSlider>(Vec(x, y), module, id);
            s->box.size = Vec(w, h); s->label = lbl; addParam(s);
        };
        auto addWave = [&](int id, float x, float y, float w, float h, std::string lbl, std::vector<int> group) {
            WaveBtn* b = createParam<WaveBtn>(Vec(x, y), module, id);
            b->box.size = Vec(w, h); b->label = lbl; b->groupIds = group; addParam(b);
        };
        auto addTgl = [&](int id, float x, float y, float w, float h, std::string lbl) {
            ToggleBtn* b = createParam<ToggleBtn>(Vec(x, y), module, id);
            b->box.size = Vec(w, h); b->label = lbl; addParam(b);
        };

        // Osc 1 Waves
        std::vector<int> grp1 = {Kocmoc::W1_SAW, Kocmoc::W1_PLS, Kocmoc::W1_SIN, Kocmoc::W1_NSE};
        addWave(Kocmoc::W1_SAW, 0, 0, 50, 31.25, "Saw", grp1);
        addWave(Kocmoc::W1_PLS, 50, 0, 50, 31.25, "Pulse", grp1);
        addWave(Kocmoc::W1_SIN, 0, 31.25, 50, 31.25, "Sin", grp1);
        addWave(Kocmoc::W1_NSE, 50, 31.25, 50, 31.25, "Noise", grp1);
        
        addRect(Kocmoc::PW1, 0, 62.5, 100, 62.5, "pulsewidth");
        addRect(Kocmoc::CRS1, 100, 0, 100, 62.5, "coarse tune");
        addRect(Kocmoc::DET1, 100, 62.5, 100, 62.5, "detune");
        addRect(Kocmoc::LVL1, 200, 0, 50, 125, "level");

        // Osc 2 Waves
        std::vector<int> grp2 = {Kocmoc::W2_SAW, Kocmoc::W2_PLS, Kocmoc::W2_SIN, Kocmoc::W2_NSE};
        addWave(Kocmoc::W2_SAW, 0, 125, 50, 31.25, "Saw", grp2);
        addWave(Kocmoc::W2_PLS, 50, 125, 50, 31.25, "Pulse", grp2);
        addWave(Kocmoc::W2_SIN, 0, 156.25, 50, 31.25, "Sin", grp2);
        addWave(Kocmoc::W2_NSE, 50, 156.25, 50, 31.25, "Noise", grp2);
        
        addRect(Kocmoc::PW2, 0, 187.5, 100, 62.5, "pulsewidth");
        addRect(Kocmoc::CRS2, 100, 125, 100, 62.5, "coarse tune");
        addRect(Kocmoc::DET2, 100, 187.5, 100, 62.5, "detune");
        addRect(Kocmoc::LVL2, 200, 125, 50, 125, "level");

        // Filters
        ArcSlider* cut = createParam<ArcSlider>(Vec(250, 0), module, Kocmoc::CUTOFF); cut->box.size=Vec(125,125); cut->label="cutoff"; addParam(cut);
        ArcSlider* res = createParam<ArcSlider>(Vec(375, 0), module, Kocmoc::RESO); res->box.size=Vec(125,125); res->label="reso"; addParam(res);

        // CV Inputs Middle
        float mx[] = {275, 325, 375, 425, 475};
        addInput(createInput<PJ301MPort>(Vec(mx[0]-12, 140-12), module, Kocmoc::PW1_CV)); addChild(new LabelWidget(Vec(mx[0], 160), "PW1"));
        addInput(createInput<PJ301MPort>(Vec(mx[1]-12, 140-12), module, Kocmoc::CRS1_CV)); addChild(new LabelWidget(Vec(mx[1], 160), "CRS1"));
        addInput(createInput<PJ301MPort>(Vec(mx[2]-12, 140-12), module, Kocmoc::DET1_CV)); addChild(new LabelWidget(Vec(mx[2], 160), "DET1"));
        addInput(createInput<PJ301MPort>(Vec(mx[3]-12, 140-12), module, Kocmoc::LVL1_CV)); addChild(new LabelWidget(Vec(mx[3], 160), "LVL1"));
        addInput(createInput<PJ301MPort>(Vec(mx[4]-12, 140-12), module, Kocmoc::ENV1_CV)); addChild(new LabelWidget(Vec(mx[4], 160), "ENV1"));

        addInput(createInput<PJ301MPort>(Vec(mx[0]-12, 180-12), module, Kocmoc::PW2_CV)); addChild(new LabelWidget(Vec(mx[0], 200), "PW2"));
        addInput(createInput<PJ301MPort>(Vec(mx[1]-12, 180-12), module, Kocmoc::CRS2_CV)); addChild(new LabelWidget(Vec(mx[1], 200), "CRS2"));
        addInput(createInput<PJ301MPort>(Vec(mx[2]-12, 180-12), module, Kocmoc::DET2_CV)); addChild(new LabelWidget(Vec(mx[2], 200), "DET2"));
        addInput(createInput<PJ301MPort>(Vec(mx[3]-12, 180-12), module, Kocmoc::LVL2_CV)); addChild(new LabelWidget(Vec(mx[3], 200), "LVL2"));
        addInput(createInput<PJ301MPort>(Vec(mx[4]-12, 180-12), module, Kocmoc::ENV2_CV)); addChild(new LabelWidget(Vec(mx[4], 200), "ENV2"));
        
        // Sequencer Outputs & KOCMOC Label in middle empty space
        addOutput(createOutput<PJ301MPort>(Vec(mx[0]-12, 220-12), module, Kocmoc::VOCT_OUT)); addChild(new LabelWidget(Vec(mx[0], 240), "V/OCT"));
        addOutput(createOutput<PJ301MPort>(Vec(mx[1]-12, 220-12), module, Kocmoc::CLK_OUT)); addChild(new LabelWidget(Vec(mx[1], 240), "CLK OUT"));
        addChild(new LabelWidget(Vec(455, 240), "KOCMOC", 18));

        // ADSRs
        addRect(Kocmoc::A1, 500, 0, 62.5, 125, "a"); addRect(Kocmoc::D1, 562.5, 0, 62.5, 125, "d");
        addRect(Kocmoc::S1, 625, 0, 62.5, 125, "s"); addRect(Kocmoc::R1, 687.5, 0, 62.5, 125, "r");
        
        addRect(Kocmoc::A2, 500, 125, 50, 125, "a"); addRect(Kocmoc::D2, 550, 125, 50, 125, "d");
        addRect(Kocmoc::S2, 600, 125, 50, 125, "s"); addRect(Kocmoc::R2, 650, 125, 50, 125, "r");
        addRect(Kocmoc::AMT, 700, 125, 50, 125, "amt");

        // Sequencer
        for (int i = 0; i < 16; i++) {
            SeqCell* cl = createParam<SeqCell>(Vec(i * 46.875f, 250), module, Kocmoc::SEQ_NOTE_0 + i);
            cl->box.size = Vec(46.875f, 50); 
            cl->stepIndex = i; // Assign properly for toggling active logic
            addParam(cl);
        }

        // Bottom Left
        addTgl(Kocmoc::PLAY, 0, 300, 62.5, 40, "play"); 
        addTgl(Kocmoc::REVERSE, 62.5, 300, 62.5, 40, "reverse");
        addRect(Kocmoc::TEMPO, 0, 340, 125, 40, "tempo");

        // Bottom Ports
        float bx[] = {150, 200, 250, 300, 350};
        addInput(createInput<PJ301MPort>(Vec(bx[0]-12, 320-12), module, Kocmoc::VOCT_IN)); addChild(new LabelWidget(Vec(bx[0], 340), "VOCT"));
        addInput(createInput<PJ301MPort>(Vec(bx[1]-12, 320-12), module, Kocmoc::CLK_IN));  addChild(new LabelWidget(Vec(bx[1], 340), "CLK"));
        addOutput(createOutput<PJ301MPort>(Vec(bx[2]-12, 320-12), module, Kocmoc::OUT_L)); addChild(new LabelWidget(Vec(bx[2], 340), "OUT L"));
        addOutput(createOutput<PJ301MPort>(Vec(bx[3]-12, 320-12), module, Kocmoc::OUT_R)); addChild(new LabelWidget(Vec(bx[3], 340), "OUT R"));
        addInput(createInput<PJ301MPort>(Vec(bx[4]-12, 320-12), module, Kocmoc::DMIX_CV)); addChild(new LabelWidget(Vec(bx[4], 340), "D.MIX"));

        addInput(createInput<PJ301MPort>(Vec(bx[0]-12, 360-12), module, Kocmoc::DTIME_CV));addChild(new LabelWidget(Vec(bx[0], 380), "D.TIME"));
        addInput(createInput<PJ301MPort>(Vec(bx[1]-12, 360-12), module, Kocmoc::DFB_CV));  addChild(new LabelWidget(Vec(bx[1], 380), "D.FB"));
        addInput(createInput<PJ301MPort>(Vec(bx[2]-12, 360-12), module, Kocmoc::RMIX_CV)); addChild(new LabelWidget(Vec(bx[2], 380), "R.MIX"));
        addInput(createInput<PJ301MPort>(Vec(bx[3]-12, 360-12), module, Kocmoc::RSIZE_CV));addChild(new LabelWidget(Vec(bx[3], 380), "R.SIZE"));
        addInput(createInput<PJ301MPort>(Vec(bx[4]-12, 360-12), module, Kocmoc::RDAMP_CV));addChild(new LabelWidget(Vec(bx[4], 380), "R.DAMP"));

        // Effects
        addRect(Kocmoc::D_MIX, 375, 300, 62.5, 80, "delay mix");
        addRect(Kocmoc::D_TIME, 437.5, 300, 62.5, 80, "delay time");
        addRect(Kocmoc::D_FB, 500, 300, 62.5, 80, "feedback");
        addRect(Kocmoc::R_MIX, 562.5, 300, 62.5, 80, "rev. mix");
        addRect(Kocmoc::R_SIZE, 625, 300, 62.5, 80, "room size");
        addRect(Kocmoc::R_DAMP, 687.5, 300, 62.5, 80, "damp");
    }
};

Model* modelKocmoc = createModel<Kocmoc, KocmocWidget>("Kocmoc");