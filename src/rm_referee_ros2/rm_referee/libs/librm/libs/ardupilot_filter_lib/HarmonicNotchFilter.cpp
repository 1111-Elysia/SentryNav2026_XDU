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

#ifndef HAL_DEBUG_BUILD
#define AP_INLINE_VECTOR_OPS
#pragma GCC optimize("O2")
#endif

#include "HarmonicNotchFilter.h"
#include <algorithm>
#include <new>
#include <cstring>

/*
  cutoff proportion of sample rate above which we do not use the notch
 */
#define HARMONIC_NYQUIST_CUTOFF 0.48f

/*
  point at which the harmonic notch goes to zero attenuation
 */
#define NOTCHFILTER_ATTENUATION_CUTOFF 0.25


/*
  destroy all of the associated notch filters
 */
template <class T>
HarmonicNotchFilter<T>::~HarmonicNotchFilter() {
    delete[] _filters;
    _num_filters = 0;
    _num_enabled_filters = 0;
}

/*
  initialise the associated filters with the provided shaping constraints
  the constraints are used to determine attenuation (A) and quality (Q) factors for the filter
 */
template <class T>
void HarmonicNotchFilter<T>::init(float sample_freq_hz, float center_freq_hz, float bandwidth_hz, float attenuation_dB)
{
    // sanity check the input
    if (_filters == nullptr || is_zero(sample_freq_hz) || std::isnan(sample_freq_hz)) {
        return;
    }

    _sample_freq_hz = sample_freq_hz;

    const float nyquist_limit = sample_freq_hz * HARMONIC_NYQUIST_CUTOFF;
    const float bandwidth_limit = bandwidth_hz * 0.52f;

    // remember the lowest frequency we will have a notch enabled at
    _minimum_freq = center_freq_hz;  // Simplified: use center_freq as minimum

    /*
      adjust the fundamental center frequency we use for the initial
      calculation of A and Q to be in the allowable range
    */
    center_freq_hz = constrain_float(center_freq_hz, bandwidth_limit, nyquist_limit);

    // Calculate spread required to achieve an equivalent single notch using two notches with Bandwidth/2
    _notch_spread = bandwidth_hz / (32 * center_freq_hz);

    // position the individual notches so that the attenuation is no worse than a single notch
    // calculate attenuation and quality from the shaping constraints
    NotchFilter<T>::calculate_A_and_Q(center_freq_hz, bandwidth_hz / _composite_notches, attenuation_dB, _A, _Q);

    _initialised = true;

    // Update filters with initial frequency
    update(center_freq_hz);
}

/*
  allocate a collection of, at most HNF_MAX_FILTERS, notch filters to be managed by this harmonic notch filter
 */
template <class T>
void HarmonicNotchFilter<T>::allocate_filters(uint8_t num_notches, uint32_t harmonics, uint8_t composite_notches)
{
    _composite_notches = std::min(composite_notches, (uint8_t)3);
    _num_harmonics = __builtin_popcount(harmonics);
    _num_filters = std::min((uint16_t)(_num_harmonics * num_notches * _composite_notches), (uint16_t)HNF_MAX_FILTERS);
    _harmonics = harmonics;

    if (_num_filters > 0) {
        _filters = new (std::nothrow) NotchFilter<T>[_num_filters];
        if (_filters == nullptr) {
            // Failed to allocate filters
            _num_filters = 0;
        }
    }
}

/*
  expand the number of filters at runtime, allowing for RPM sources such as lua scripts
 */
template <class T>
void HarmonicNotchFilter<T>::expand_filter_count(uint16_t total_notches)
{
    if (total_notches <= _num_filters) {
        // enough already
        return;
    }
    if (_alloc_has_failed) {
        // we've failed to allocate before, don't try again
        return;
    }
    /*
      note that we rely on the semaphore in
      AP_InertialSensor_Backend.cpp to make this thread safe
     */
    auto filters = new (std::nothrow) NotchFilter<T>[total_notches];
    if (filters == nullptr) {
        _alloc_has_failed = true;
        return;
    }
    memcpy(filters, _filters, sizeof(filters[0])*_num_filters);
    auto _old_filters = _filters;
    _filters = filters;
    _num_filters = total_notches;
    delete[] _old_filters;
}

/*
  set the center frequency of a single notch harmonic

  The spread_mul is the frequency multiplier from the spread of the
  double or triple notch. The harmonic_mul is the multiplier for the
  frequency for this harmonic
 */
template <class T>
void HarmonicNotchFilter<T>::set_center_frequency(uint16_t idx, float notch_center, float spread_mul, uint8_t harmonic_mul)
{
    const float nyquist_limit = _sample_freq_hz * HARMONIC_NYQUIST_CUTOFF;
    auto &notch = _filters[idx];

    // scale the notch with the harmonic multiplier
    notch_center *= harmonic_mul;

    /* disable the filter if its center frequency is above the nyquist
       frequency

       NOTE: should this be notch_center*spread_mul ? As it is here we
       do sometimes run the upper notch in a double or triple notch at
       higher than the nyquist.
    */
    if (notch_center >= nyquist_limit) {
        notch.disable();
        return;
    }

    // the minimum frequency for a harmonic is the base minimum
    float harmonic_min_freq = _minimum_freq;

    // we can adjust the attenuation at low frequencies
    float A = _A;

    // Simplified: disable if notch is too low
    const float disable_freq = harmonic_min_freq * NOTCHFILTER_ATTENUATION_CUTOFF;
    if (notch_center < disable_freq) {
        notch.disable();
        return;
    }

    if (notch_center < harmonic_min_freq) {
        /*
          scale the attenuation so that we fade out the notch as
          we get further below the min frequency. The attenuation
          factor A goes to 1.0 (which means no attenuation)
          Scaling the attenuation in this way means we don't get a
          glitch at the disable point
         */
        A = _A + (_A - 1.0f) * (notch_center - harmonic_min_freq) / (harmonic_min_freq - disable_freq);
    }

    // don't let the notch go below the min frequency
    notch_center = std::max(notch_center, harmonic_min_freq);

    /* adjust notch center for spread for double and triple notch.
       This adjustment is applied last to maintain the symmetry of the
       double and triple notches
    */
    notch_center *= spread_mul;

    notch.init_with_A_and_Q(_sample_freq_hz, notch_center, A, _Q);
}

/*
  update the underlying filters' center frequency using the current attenuation and quality
  this function is cheaper than init() because A & Q do not need to be recalculated
 */
template <class T>
void HarmonicNotchFilter<T>::update(float center_freq_hz)
{
    update(1, &center_freq_hz);
}

/*
  update the underlying filters' center frequency using the current attenuation and quality
  this function is cheaper than init() because A & Q do not need to be recalculated
 */
template <class T>
void HarmonicNotchFilter<T>::update(uint8_t num_centers, const float center_freq_hz[])
{
    if (!_initialised) {
        return;
    }

    // adjust the frequencies to be in the allowable range
    const float nyquist_limit = _sample_freq_hz * HARMONIC_NYQUIST_CUTOFF;

    const uint16_t total_notches = std::min((uint16_t)(num_centers * _num_harmonics * _composite_notches), (uint16_t)HNF_MAX_FILTERS);
    if (total_notches > _num_filters) {
        // alloc realloc of filters
        expand_filter_count(total_notches);
    }

    _num_enabled_filters = 0;

    // update all of the filters using the new center frequencies and existing A & Q
    for (uint16_t i = 0; i < num_centers * HNF_MAX_HARMONICS && _num_enabled_filters < _num_filters; i++) {
        const uint8_t harmonic_n = i / num_centers;
        const uint8_t center_n = i % num_centers;
        // the filters are ordered by center and then harmonic so
        // f1h1, f2h1, f3h1, f4h1, f1h2, f2h2, etc
        if (!((1U<<harmonic_n) & _harmonics)) {
            continue;
        }

        const float notch_center = constrain_float(center_freq_hz[center_n], 0.0f, nyquist_limit);
        const uint8_t harmonic_mul = (harmonic_n+1);
        if (_composite_notches != 2) {
            set_center_frequency(_num_enabled_filters++, notch_center, 1.0, harmonic_mul);
        }
        if (_composite_notches > 1) {
            set_center_frequency(_num_enabled_filters++, notch_center, 1.0 - _notch_spread, harmonic_mul);
            set_center_frequency(_num_enabled_filters++, notch_center, 1.0 + _notch_spread, harmonic_mul);
        }
        if (_composite_notches > 3) {
            set_center_frequency(_num_enabled_filters++, notch_center, 1.0 - 2 * _notch_spread, harmonic_mul);
            set_center_frequency(_num_enabled_filters++, notch_center, 1.0 + 2 * _notch_spread, harmonic_mul);
        }
    }
}

/*
  apply a sample to each of the underlying filters in turn and return the output
 */
template <class T>
T HarmonicNotchFilter<T>::apply(const T &sample)
{
    if (!_initialised) {
        return sample;
    }

    T output = sample;
    for (uint16_t i = 0; i < _num_enabled_filters; i++) {
        output = _filters[i].apply(output);
    }
    return output;
}

/*
  reset all of the underlying filters
 */
template <class T>
void HarmonicNotchFilter<T>::reset()
{
    if (!_initialised) {
        return;
    }

    for (uint16_t i = 0; i < _num_filters; i++) {
        _filters[i].reset();
    }
}



/* 
  instantiate template classes
 */
template class HarmonicNotchFilter<Vector3f>;
template class HarmonicNotchFilter<float>;

