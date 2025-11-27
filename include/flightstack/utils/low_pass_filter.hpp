/***********************************************************************************************************************
 * Copyright (c) 2024 Mattia Gramuglia, Giri M. Kumar, Andrea L'Afflitto. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 *    disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * File:        low_pass_filter.hpp
 * Author:      Mattia Gramuglia
 * Date:        September 25, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Implementation of first and second order recursive low pass filters.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack.git
 **********************************************************************************************************************/

#pragma once

#include <cmath>

namespace LowPassFilter {

class FirstOrder {
public:
  FirstOrder() : alpha(0.0), y(0.0), initialized(false) {}

  // Configure the filter with a cutoff frequency (Hz) and sample rate (Hz)
  void configure(double cutoff_hz, double sample_rate_hz) {
    // RC time constant: determines the filter response speed
    double rc = 1.0 / (2.0 * M_PI * cutoff_hz);
    // Compute filter coefficient alpha (0 < alpha < 1)
    alpha = 1.0 / (1.0 + (rc * sample_rate_hz));
    initialized = false; // reset flag
  }

  // Process one new input sample 'x' and return the filtered output
  inline double process(double x) {
    // First-order low-pass formula:
    // y[n] = y[n-1] + alpha * (x[n] - y[n-1])
    // Moves output y toward input x by fraction alpha
    if (!initialized) {
      y = x;        // set initial output to first input
      initialized = true;
    }
    y = y + alpha * (x - y);
    return y;
  }

  // Reset filter state to a given value (default 0)
  void reset(double value = 0.0) {
    y = value;
    initialized = true;  // if you reset, we know the output
  }

private:
  double alpha; // filter coefficient controlling cutoff frequency (0 < alpha < 1)
  double y; // previous filter output (y[n-1]), also stores current output after update
  bool initialized;
};

class SecondOrder {
public:
  SecondOrder() : 
    b0(0.0), b1(0.0), b2(0.0), a1(0.0), a2(0.0),
    x1(0.0), x2(0.0), y1(0.0), y2(0.0), initialized(false) {}

  // Configure the filter with cutoff frequency (Hz), sample rate (Hz), and quality factor Q
  void configure(double cutoff_hz, double sample_rate_hz, double q = 0.7071) {
    double w0 = 2.0 * M_PI * cutoff_hz / sample_rate_hz; // normalized angular frequency
    double cos_w0 = std::cos(w0);
    double sin_w0 = std::sin(w0);
    double alpha = sin_w0 / (2.0 * q); // damping factor

    double a0_inv = 1.0 / (1.0 + alpha); // normalization factor

    // Biquad coefficients for a standard second-order low-pass filter
    b0 = ((1.0 - cos_w0) * 0.5) * a0_inv;
    b1 = (1.0 - cos_w0) * a0_inv;
    b2 = b0;
    a1 = -2.0 * cos_w0 * a0_inv;
    a2 = (1.0 - alpha) * a0_inv;

    initialized = false; // mark uninitialized for first input
  }

  // Process one input sample and return the filtered output
  inline double process(double x) {
    if (!initialized) {
      // Set all states to the first input
      x1 = x2 = y1 = y2 = x;
      initialized = true;
    }
    // Second-order difference equation:
    // y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
    double y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;

    // Update state variables for next sample
    x2 = x1;
    x1 = x;
    y2 = y1;
    y1 = y;

    return y;
  }

  // Reset filter state to a given value (default 0)
  void reset(double value = 0.0) {
    x1 = x2 = y1 = y2 = value;
    initialized = true;
  }

private:
  // Filter coefficients
  double b0, b1, b2; // numerator coefficients
  double a1, a2;     // denominator coefficients

  // State variables (previous inputs and outputs)
  double x1, x2; // x[n-1], x[n-2]
  double y1, y2; // y[n-1], y[n-2]

  bool initialized;
};

/**
 * @brief A generic 3-channel vector wrapper for digital filters.
 * 
 * This class applies a filter of type `FilterType` independently to each
 * component of a 3-dimensional vector (Eigen::Vector3d). It supports 
 * per-channel configuration of filter parameters such as cutoff frequency,
 * sample rate, and, if applicable, quality factor (Q).
 * 
 * The wrapper handles:
 *  - Per-channel configuration via the nested Config struct.
 *  - Component-wise processing of Eigen::Vector3d inputs.
 *  - Resetting all channels to a given initial value.
 * 
 * Template parameter:
 *  - FilterType: the filter class to be applied (e.g., FirstOrder, SecondOrder)
 */
template <typename FilterType>
class FilterVector3d {
public:
  // Expose the underlying filter type so we can deduce it elsewhere
  using filter_type = FilterType;

  FilterVector3d() = default;

  // Nested struct for per-channel configuration
  struct Config {
    double cutoff_hz;
    double sample_rate_hz;
    double q = 0.7071; // Only used by second-order filters
  };

  // Configure each channel individually
  void configure(const std::array<Config, 3>& configs) {
    for (int i = 0; i < 3; i++) {
      configureChannel(filters[i], configs[i]);
    }
  }

  // Process an Eigen::Vector3d input component-wise
  Eigen::Vector3d process(const Eigen::Vector3d& x) {
    Eigen::Vector3d y;
    for (int i = 0; i < 3; i++)
      y[i] = filters[i].process(x[i]);
    return y;
  }

  void reset(double init = 0.0) {
    for (auto& f : filters)
      f.reset(init);
  }

private:
  std::array<FilterType, 3> filters;

  template <typename F = FilterType>
  void configureChannel(F& f, const Config& cfg) {
    if constexpr (std::is_same_v<F, FirstOrder>) {
      f.configure(cfg.cutoff_hz, cfg.sample_rate_hz);
    } else if constexpr (std::is_same_v<F, SecondOrder>) {
      f.configure(cfg.cutoff_hz, cfg.sample_rate_hz, cfg.q);
    }
  }
};


} // namespace LowPassFilter
