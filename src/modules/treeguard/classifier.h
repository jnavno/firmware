#pragma once

#include <Arduino.h>
struct Features {
    float hf_energy;
    float fft_0_25_ratio;
    float fft_125_200_ratio;
    int strike_count;
    float offset_x_y;
    float offset_y_z;
    float max_y;
    float max_z;
};

String classifyBufferedData(float *x_buffer, float *y_buffer, float *z_buffer, int num_samples, Features *out = nullptr);
