/*
  classifier.cpp
  ====== Vibration Classification with Feature Export ======

  Changes from previous version:
  1. Added `Features` struct to header for optional feature capture.
  2. Expanded `classifyBufferedData` signature to accept an optional `Features* out` parameter (default nullptr) for backwards
  compatibility.
  3. Refactored in-function variable names to match original (accelX_buffer, accelY_buffer, accelZ_buffer).
  4. Consolidated feature computation in one place, populating `out` if provided.
  5. Removed verbose Serial feature-summary prints (now optional via returned `out`).
*/

#include "modules/treeguard/fft/FFT.h"
#include "modules/treeguard/fft/FFT_signal.h"
#include "modules/treeguard/fft/fft_config.h"

#include "modules/treeguard/classifier.h"
#include <math.h>

#define SAMPLING_FREQUENCY 333 // Hz

String classifyBufferedData(float *accelX_buffer, float *accelY_buffer, float *accelZ_buffer, int num_samples, Features *out)
{
    // 1) Compute mean for DC removal
    float meanX = 0, meanY = 0, meanZ = 0;
    for (int i = 0; i < num_samples; i++) {
        meanX += accelX_buffer[i];
        meanY += accelY_buffer[i];
        meanZ += accelZ_buffer[i];
    }
    meanX /= num_samples;
    meanY /= num_samples;
    meanZ /= num_samples;

    // 2) Populate FFT input as magnitude of centered accel vector
    for (int i = 0; i < num_samples; i++) {
        float x = accelX_buffer[i] - meanX;
        float y = accelY_buffer[i] - meanY;
        float z = accelZ_buffer[i] - meanZ;
        fft_input[i] = sqrt(x * x + y * y + z * z);
    }

    // 3) Execute FFT
    fft_config_t *fft_plan = fft_init(FFT_N, FFT_REAL, FFT_FORWARD, fft_input, fft_output);
    if (!fft_plan) {
        Serial.println("[ERROR] FFT plan allocation failed!");
        return "⚠️ Unclassified";
    }
    fft_execute(fft_plan);
    fft_destroy(fft_plan);

    // 4) Extract spectral features
    double total_energy = 0;
    double band_0_25 = 0;
    double band_125_200 = 0;
    for (int i = 1; i < FFT_N / 2; i++) {
        float re = fft_output[2 * i];
        float im = fft_output[2 * i + 1];
        float mag = sqrt(re * re + im * im);
        float freq = i * (float)SAMPLING_FREQUENCY / FFT_N;
        total_energy += mag;
        if (freq <= 25)
            band_0_25 += mag;
        else if (freq >= 125 && freq <= 200)
            band_125_200 += mag;
    }
    float fft_0_25_ratio = total_energy > 0 ? band_0_25 / total_energy : 0;
    float fft_125_200_ratio = total_energy > 0 ? band_125_200 / total_energy : 0;

    // 5) Count time-domain peaks (strike_count)
    int strike_count = 0;
    for (int i = 1; i < num_samples - 1; i++) {
        if (fft_input[i] > 5000 && fft_input[i] > fft_input[i - 1] && fft_input[i] > fft_input[i + 1]) {
            strike_count++;
        }
    }

    // 6) Compute offsets and max magnitudes
    float offset_x_y = 0, offset_y_z = 0;
    float max_y = 0, max_z = 0;
    for (int i = 0; i < num_samples; i++) {
        offset_x_y += (accelX_buffer[i] - accelY_buffer[i]);
        offset_y_z += (accelY_buffer[i] - accelZ_buffer[i]);
        max_y = max(max_y, fabs(accelY_buffer[i]));
        max_z = max(max_z, fabs(accelZ_buffer[i]));
    }
    offset_x_y /= num_samples;
    offset_y_z /= num_samples;

    // 7) Compute high-frequency energy ratio
    float hf_energy = 0;
    for (int i = FFT_N / 8; i < FFT_N / 2; i++) {
        float re = fft_output[2 * i];
        float im = fft_output[2 * i + 1];
        hf_energy += sqrt(re * re + im * im);
    }
    hf_energy /= total_energy > 0 ? total_energy : 1;

    // 8) Populate output struct if caller provided one
    if (out) {
        out->hf_energy = hf_energy;
        out->fft_0_25_ratio = fft_0_25_ratio;
        out->fft_125_200_ratio = fft_125_200_ratio;
        out->strike_count = strike_count;
        out->offset_x_y = offset_x_y;
        out->offset_y_z = offset_y_z;
        out->max_y = max_y;
        out->max_z = max_z;
    }

    // 9) Decision tree, unchanged logic
    if (strike_count >= 1 && strike_count <= 100 && hf_energy >= 0.4 && fft_0_25_ratio > 0.3 && fft_125_200_ratio < 0.22 &&
        offset_x_y >= 16000 && fabs(offset_y_z) <= 1200 && max_y > 15000 && max_z > 15000)
        return "✅ Likely Machete";

    else if (strike_count > 200 && hf_energy > 0.6 && fft_0_25_ratio < 0.3 && fft_125_200_ratio > 0.15 &&
             (max_y > 3000 || max_z > 3000))
        return "🔧 Likely Chainsaw";

    else if (max_y < 2000 && max_z < 2000 && offset_x_y > 16000 && fft_125_200_ratio > 0.12 && fft_125_200_ratio < 0.27 &&
             hf_energy > 0.7 && strike_count > 600)
        return "🌬️ Likely Non-Event";

    else if (strike_count >= 70 && strike_count <= 110 && hf_energy > 0.45 && fft_0_25_ratio >= 0.22 &&
             fft_125_200_ratio < 0.27 && offset_x_y >= 15300 && fabs(offset_y_z) <= 1300 && (max_y > 15000 || max_z > 15000))
        return "🟡 Possibly Machete";

    else if (fft_125_200_ratio > 0.28 && hf_energy > 0.75 && strike_count < 300 && max_y > 10000 && max_z > 10000)
        return "⚠️ Possibly Chainsaw (Low Activity)";

    else if (hf_energy > 0.5 && strike_count > 20 && (max_y > 1000 || max_z > 1000))
        return "⚠️ Vibration Detected (ambiguous)";

    return "⚠️ Unclassified";
}
