/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include "filter_math.h"
#include "filter_config.h"
#include <cmath>
#include "NotchFilter.h"

/*
  a filter that manages a set of notch filters targetted at a fundamental center frequency
  and multiples of that fundamental frequency
 */
template <class T>
class HarmonicNotchFilter {
public:
    ~HarmonicNotchFilter();
    // allocate a bank of notch filters for this harmonic notch filter
    void allocate_filters(uint8_t num_notches, uint32_t harmonics, uint8_t composite_notches);
    // expand filter bank with new filters
    void expand_filter_count(uint16_t total_notches);
    // initialize the underlying filters - simplified standalone version
    void init(float sample_freq_hz, float center_freq_hz, float bandwidth_hz, float attenuation_dB);
    // update the underlying filters' center frequencies using center_freq_hz as the fundamental
    void update(float center_freq_hz);
    // update all of the underlying center frequencies individually
    void update(uint8_t num_centers, const float center_freq_hz[]);

    /*
      set center frequency of one notch.
      spread_mul is a scale factor for spreading of double or triple notch
      harmonic_mul is the multiplier for harmonics, 1 is for the fundamental
    */
    void set_center_frequency(uint16_t idx, float center_freq_hz, float spread_mul, uint8_t harmonic_mul);

    // apply a sample to each of the underlying filters in turn
    T apply(const T &sample);
    // reset each of the underlying filters
    void reset();

private:
    // underlying bank of notch filters
    NotchFilter<T>*  _filters;
    // sample frequency for each filter
    float _sample_freq_hz;
    // base double notch bandwidth for each filter
    float _notch_spread;
    // attenuation for each filter
    float _A;
    // quality factor of each filter
    float _Q;
    // a bitmask of the harmonics to use
    uint32_t _harmonics;
    // number of notches that make up a composite notch
    uint8_t _composite_notches;
    // number of allocated filters
    uint16_t _num_filters;
    // pre-calculated number of harmonics
    uint8_t _num_harmonics;
    // number of enabled filters
    uint16_t _num_enabled_filters;
    bool _initialised;

    // have we failed to expand filters?
    bool _alloc_has_failed;

    // minimum frequency
    float _minimum_freq;
};

typedef HarmonicNotchFilter<Vector3f> HarmonicNotchFilterVector3f;

