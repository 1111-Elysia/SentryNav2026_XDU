#pragma once

// Standalone filter configuration
// User can override these by defining them before including filter headers

#ifndef AP_FILTER_ENABLED
#define AP_FILTER_ENABLED 0  // Disable AP_Filter framework (parameter-based config)
#endif

#ifndef AP_FILTER_NUM_FILTERS
#define AP_FILTER_NUM_FILTERS 4
#endif

// Maximum number of harmonics for harmonic notch filter
#ifndef HNF_MAX_HARMONICS
#define HNF_MAX_HARMONICS 16
#endif

// Maximum number of notches per harmonic notch filter
#ifndef HNF_MAX_FILTERS
#define HNF_MAX_FILTERS 12
#endif

