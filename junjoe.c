#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

#define PI 3.14159265358979323846f
#define NUM_VOICES 6
#define NUM_PARAMS 8

struct AEffect;
typedef intptr_t (*audioMasterCallback)(struct AEffect*, int32_t, int32_t, intptr_t, void*, float);

typedef struct AEffect {
    int32_t magic;
    intptr_t (*dispatcher)(struct AEffect*, int32_t, int32_t, intptr_t, void*, float);
    void (*process)(struct AEffect*, float**, float**, int32_t);
    void (*setParameter)(struct AEffect*, int32_t, float);
    float (*getParameter)(struct AEffect*, int32_t);
    int32_t numPrograms, numParams, numInputs, numOutputs, flags;
    intptr_t resvd1, resvd2;
    int32_t initialDelay, realQualities, offQualities;
    float ioRatio;
    void* object;
    void* user;
    int32_t uniqueID, version;
    void (*processReplacing)(struct AEffect*, float**, float**, int32_t);
    void* processDoubleReplacing;
    char future[56];
} AEffect;

typedef struct {
    int32_t type, byteSize, deltaFrames, flags, noteLength, noteOffset;
    char midiData[4], detune, noteOffVelocity, reserved1, reserved2;
} VstMidiEvent;

typedef struct {
    int32_t numEvents;
    intptr_t reserved;
    void* events[2];
} VstEvents;

typedef struct {
    uint8_t note;
    int active;
    uint32_t age;
    float phase, sub_phase;
    float env_val;
    int env_stage; // 0: off, 1: attack, 2: release
    float y[4];    // 4-pole filter state
} Voice;

typedef struct {
    float sample_rate;
    float params[NUM_PARAMS];
    Voice voices[NUM_VOICES];
    uint32_t age_counter;
    float chorus_buf_l[4410];
    float chorus_buf_r[4410];
    int chorus_idx;
    float chorus_phase;
} Kr106Synth;

static inline float poly_blep(float t, float dt) {
    if (t < dt) { t /= dt; return 2.0f * t - t * t - 1.0f; }
    if (t > 1.0f - dt) { t = (t - 1.0f) / dt; return t * t + 2.0f * t + 1.0f; }
    return 0.0f;
}

static inline float fast_tanh(float x) {
    float x2 = x * x;
    return x * (27.0f + x2) / (27.0f + 9.0f * x2);
}

static void note_on(Kr106Synth* s, uint8_t note) {
    s->age_counter++;
    Voice* target = NULL;
    for (int i = 0; i < NUM_VOICES; i++) {
        if (!s->voices[i].active) { target = &s->voices[i]; break; }
    }
    if (!target) {
        target = &s->voices[0];
        for (int i = 1; i < NUM_VOICES; i++) {
            if (s->voices[i].age < target->age) target = &s->voices[i];
        }
    }
    target->note = note;
    target->active = 1;
    target->age = s->age_counter;
    target->phase = 0.0f;
    target->sub_phase = 0.0f;
    target->env_stage = 1;
    memset(target->y, 0, sizeof(target->y));
}

static void note_off(Kr106Synth* s, uint8_t note) {
    for (int i = 0; i < NUM_VOICES; i++) {
        if (s->voices[i].active && s->voices[i].note == note) {
            s->voices[i].env_stage = 2;
        }
    }
}

static void process_replacing(AEffect* effect, float** inputs, float** outputs, int32_t sampleFrames) {
    Kr106Synth* s = (Kr106Synth*)effect->user;
    float* out_l = outputs[0];
    float* out_r = outputs[1];

    float saw_lvl = s->params[0];
    float pulse_lvl = s->params[1];
    float sub_lvl = s->params[2];
    float cutoff_base = 20.0f * powf(1000.0f, s->params[3]);
    float res = s->params[4] * 3.8f;
    float attack_inc = 1.0f / (fmaxf(s->params[5] * 2.0f, 0.005f) * s->sample_rate);
    float release_dec = 1.0f / (fmaxf(s->params[6] * 3.0f, 0.01f) * s->sample_rate);
    int chorus_on = s->params[7] > 0.5f;

    for (int i = 0; i < sampleFrames; i++) {
        float mix = 0.0f;

        for (int v = 0; v < NUM_VOICES; v++) {
            Voice* vc = &s->voices[v];
            if (!vc->active) continue;

            if (vc->env_stage == 1) {
                vc->env_val += attack_inc;
                if (vc->env_val >= 1.0f) { vc->env_val = 1.0f; vc->env_stage = 0; }
            } else if (vc->env_stage == 2) {
                vc->env_val -= release_dec;
                if (vc->env_val <= 0.0f) { vc->env_val = 0.0f; vc->active = 0; continue; }
            }

            float freq = 440.0f * powf(2.0f, (vc->note - 69.0f) / 12.0f);
            float dt = freq / s->sample_rate;
            float sub_dt = (freq * 0.5f) / s->sample_rate;

            float saw = (2.0f * vc->phase - 1.0f) - poly_blep(vc->phase, dt);
            float pulse = (vc->phase < 0.5f ? 1.0f : -1.0f) + poly_blep(vc->phase, dt) - poly_blep(fmodf(vc->phase + 0.5f, 1.0f), dt);
            float sub = (vc->sub_phase < 0.5f ? 1.0f : -1.0f) + poly_blep(vc->sub_phase, sub_dt) - poly_blep(fmodf(vc->sub_phase + 0.5f, 1.0f), sub_dt);

            vc->phase = fmodf(vc->phase + dt, 1.0f);
            vc->sub_phase = fmodf(vc->sub_phase + sub_dt, 1.0f);

            float voice_sig = saw * saw_lvl + pulse * pulse_lvl + sub * sub_lvl;

            float f = fminf((cutoff_base + vc->env_val * 4000.0f) / s->sample_rate, 0.45f);
            float g = 1.0f - expf(-2.0f * PI * f);
            float fb = fast_tanh(voice_sig - res * vc->y[3]);

            vc->y[0] += g * (fb - vc->y[0]);
            vc->y[1] += g * (vc->y[0] - vc->y[1]);
            vc->y[2] += g * (vc->y[1] - vc->y[2]);
            vc->y[3] += g * (vc->y[2] - vc->y[3]);

            mix += vc->y[3] * vc->env_val;
        }

        mix *= 0.25f;

        if (chorus_on) {
            s->chorus_buf_l[s->chorus_idx] = mix;
            s->chorus_buf_r[s->chorus_idx] = mix;

            float mod = sinf(s->chorus_phase * 2.0f * PI);
            s->chorus_phase = fmodf(s->chorus_phase + 0.6f / s->sample_rate, 1.0f);

            float d_l = (0.005f + 0.002f * mod) * s->sample_rate;
            float d_r = (0.005f - 0.002f * mod) * s->sample_rate;

            int read_l = (s->chorus_idx - (int)d_l + 4410) % 4410;
            int read_r = (s->chorus_idx - (int)d_r + 4410) % 4410;

            out_l[i] = mix * 0.6f + s->chorus_buf_l[read_l] * 0.5f;
            out_r[i] = mix * 0.6f + s->chorus_buf_r[read_r] * 0.5f;

            s->chorus_idx = (s->chorus_idx + 1) % 4410;
        } else {
            out_l[i] = mix;
            out_r[i] = mix;
        }
    }
}

static void set_parameter(AEffect* effect, int32_t index, float value) {
    Kr106Synth* s = (Kr106Synth*)effect->user;
    if (index >= 0 && index < NUM_PARAMS) s->params[index] = value;
}

static float get_parameter(AEffect* effect, int32_t index) {
    Kr106Synth* s = (Kr106Synth*)effect->user;
    return (index >= 0 && index < NUM_PARAMS) ? s->params[index] : 0.0f;
}

static intptr_t dispatcher(AEffect* effect, int32_t opcode, int32_t index, intptr_t value, void* ptr, float opt) {
    Kr106Synth* s = (Kr106Synth*)effect->user;
    switch (opcode) {
        case 1: if (s) free(s); free(effect); return 0; // effClose
        case 6: // effGetParamLabel
            if (index == 3) strcpy((char*)ptr, "Hz");
            else if (index == 5 || index == 6) strcpy((char*)ptr, "s");
            else strcpy((char*)ptr, "%");
            return 0;
        case 7: // effGetParamDisplay
            if (index == 3) sprintf((char*)ptr, "%.0f", 20.0f * powf(1000.0f, s->params[3]));
            else if (index == 7) strcpy((char*)ptr, s->params[7] > 0.5f ? "On" : "Off");
            else sprintf((char*)ptr, "%.2f", s->params[index]);
            return 0;
        case 8: // effGetParamName
            switch (index) {
                case 0: strcpy((char*)ptr, "Saw Level"); break;
                case 1: strcpy((char*)ptr, "Pulse Level"); break;
                case 2: strcpy((char*)ptr, "Sub Level"); break;
                case 3: strcpy((char*)ptr, "Cutoff"); break;
                case 4: strcpy((char*)ptr, "Resonance"); break;
                case 5: strcpy((char*)ptr, "Attack"); break;
                case 6: strcpy((char*)ptr, "Release"); break;
                case 7: strcpy((char*)ptr, "Chorus"); break;
            }
            return 0;
        case 10: s->sample_rate = opt; return 0; // effSetSampleRate
        case 25: { // effProcessEvents
            VstEvents* evs = (VstEvents*)ptr;
            for (int i = 0; i < evs->numEvents; i++) {
                VstMidiEvent* m = (VstMidiEvent*)evs->events[i];
                if (m->type == 1) {
                    uint8_t status = m->midiData[0] & 0xF0;
                    uint8_t note = m->midiData[1];
                    uint8_t vel = m->midiData[2];
                    if (status == 0x90 && vel > 0) note_on(s, note);
                    else if (status == 0x80 || (status == 0x90 && vel == 0)) note_off(s, note);
                }
            }
            return 1;
        }
        case 35: return 8; // kPlugCategSynth
        case 45: strcpy((char*)ptr, "KR-106"); return 0;
        case 47: strcpy((char*)ptr, "OpenAudio"); return 0;
        default: return 0;
    }
}

__declspec(dllexport) AEffect* VSTPluginMain(audioMasterCallback audioMaster) {
    AEffect* effect = (AEffect*)calloc(1, sizeof(AEffect));
    Kr106Synth* synth = (Kr106Synth*)calloc(1, sizeof(Kr106Synth));

    synth->sample_rate = 44100.0f;
    synth->params[0] = 0.8f; // Saw
    synth->params[1] = 0.4f; // Pulse
    synth->params[2] = 0.5f; // Sub
    synth->params[3] = 0.6f; // Cutoff
    synth->params[4] = 0.3f; // Res
    synth->params[5] = 0.05f; // Attack
    synth->params[6] = 0.4f; // Release
    synth->params[7] = 1.0f; // Chorus On

    effect->magic = 0x56737450; // 'VstP'
    effect->dispatcher = dispatcher;
    effect->setParameter = set_parameter;
    effect->getParameter = get_parameter;
    effect->processReplacing = process_replacing;
    effect->numPrograms = 1;
    effect->numParams = NUM_PARAMS;
    effect->numInputs = 0;
    effect->numOutputs = 2;
    effect->flags = (1 << 4) | (1 << 8); // CanReplacing | IsSynth
    effect->uniqueID = 106106;
    effect->user = synth;

    return effect;
}
