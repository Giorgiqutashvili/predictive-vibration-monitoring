/*
 * dsp_features.hpp
 *
 *  Created on: Aug 24, 2026
 *      Author: lukam
 */

#ifndef INC_DSP_FEATURES_HPP_
#define INC_DSP_FEATURES_HPP_

#pragma once

#include <cmath>
#include <cstddef>
#include "ring_buffer.hpp"

// Structure holding statistical vibration indicators for one axis
struct AxisFeatures {
    float mean;         // DC bias / gravity component (mg)
    float rms;          // Vibration energy (mg RMS)
    float peak_to_peak; // Total deflection span (mg)
    float crest_factor; // Impulsiveness ratio (Peak / RMS)
};

// Structure holding features for all 3 spatial axes
struct VibrationFeatures {
    AxisFeatures x;
    AxisFeatures y;
    AxisFeatures z;
};

class FeatureExtractor {
public:
    // Computes statistical indicators over an array of vibration samples
    static VibrationFeatures compute(const VibrationSample *samples, size_t count) {
        VibrationFeatures feat = {};
        if (count == 0 || samples == nullptr) {
            return feat;
        }

        // 1. Calculate DC Means
        double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
        for (size_t i = 0; i < count; i++) {
            sum_x += samples[i].x;
            sum_y += samples[i].y;
            sum_z += samples[i].z;
        }
        feat.x.mean = static_cast<float>(sum_x / count);
        feat.y.mean = static_cast<float>(sum_y / count);
        feat.z.mean = static_cast<float>(sum_z / count);

        // 2. Calculate Peak-to-Peak, AC Energy (Variance), and Max Absolute Peak
        float min_x = samples[0].x, max_x = samples[0].x, max_abs_x = 0.0f;
        float min_y = samples[0].y, max_y = samples[0].y, max_abs_y = 0.0f;
        float min_z = samples[0].z, max_z = samples[0].z, max_abs_z = 0.0f;

        double sum_sq_ac_x = 0.0, sum_sq_ac_y = 0.0, sum_sq_ac_z = 0.0;

        for (size_t i = 0; i < count; i++) {
            // Remove DC bias to analyze dynamic vibration only
            float ac_x = samples[i].x - feat.x.mean;
            float ac_y = samples[i].y - feat.y.mean;
            float ac_z = samples[i].z - feat.z.mean;

            sum_sq_ac_x += ac_x * ac_x;
            sum_sq_ac_y += ac_y * ac_y;
            sum_sq_ac_z += ac_z * ac_z;

            // Track min / max for Peak-to-Peak
            if (samples[i].x < min_x) min_x = samples[i].x;
            if (samples[i].x > max_x) max_x = samples[i].x;
            if (samples[i].y < min_y) min_y = samples[i].y;
            if (samples[i].y > max_y) max_y = samples[i].y;
            if (samples[i].z < min_z) min_z = samples[i].z;
            if (samples[i].z > max_z) max_z = samples[i].z;

            // Track maximum peak magnitude
            float abs_x = std::fabs(ac_x);
            float abs_y = std::fabs(ac_y);
            float abs_z = std::fabs(ac_z);
            if (abs_x > max_abs_x) max_abs_x = abs_x;
            if (abs_y > max_abs_y) max_abs_y = abs_y;
            if (abs_z > max_abs_z) max_abs_z = abs_z;
        }

        // 3. Compute final metrics
        feat.x.rms = static_cast<float>(std::sqrt(sum_sq_ac_x / count));
        feat.y.rms = static_cast<float>(std::sqrt(sum_sq_ac_y / count));
        feat.z.rms = static_cast<float>(std::sqrt(sum_sq_ac_z / count));

        feat.x.peak_to_peak = max_x - min_x;
        feat.y.peak_to_peak = max_y - min_y;
        feat.z.peak_to_peak = max_z - min_z;

        // Crest Factor = Peak / RMS (avoid divide by zero)
        feat.x.crest_factor = (feat.x.rms > 0.001f) ? (max_abs_x / feat.x.rms) : 0.0f;
        feat.y.crest_factor = (feat.y.rms > 0.001f) ? (max_abs_y / feat.y.rms) : 0.0f;
        feat.z.crest_factor = (feat.z.rms > 0.001f) ? (max_abs_z / feat.z.rms) : 0.0f;

        return feat;
    }
};



#endif /* INC_DSP_FEATURES_HPP_ */
