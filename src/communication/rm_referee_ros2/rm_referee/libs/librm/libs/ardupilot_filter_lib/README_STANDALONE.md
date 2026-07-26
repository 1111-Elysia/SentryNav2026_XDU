# ArduPilot Filter Library - Standalone Version

This is a standalone version of the ArduPilot filter library, extracted and adapted for use outside of the ArduPilot ecosystem.

## Changes from Original

The following dependencies have been removed to make this library standalone:

1. **AP_Param** - Parameter management framework (not needed for direct filter usage)
2. **AP_HAL** - Hardware abstraction layer (replaced with minimal `filter_hal::millis()`)
3. **AP_Math** - Math utilities (replaced with `filter_math.h` containing Vector2f/Vector3f and utilities)
4. **GCS_MAVLink** - Telemetry and logging (removed)
5. **AP_Logger** - Logging framework (removed)

## Files Excluded

- `AP_Filter.cpp` / `AP_Filter.h` - Parameter-based filter framework (requires AP_Param)
- `AP_Filter_params.cpp` - Parameter definitions
- `AP_NotchFilter_params.cpp` - Notch filter parameters
- `AP_Filter_config.h` - Original config (replaced with `filter_config.h`)

## Available Filter Classes

### Low Pass Filters
- **LowPassFilter** - Variable time step low pass filter
- **LowPassFilterConstDt** - Constant time step low pass filter (more efficient)
- **LowPassFilter2p** - Second-order (biquad) low pass filter

### Notch Filters
- **NotchFilter** - Single notch filter
- **HarmonicNotchFilter** - Multi-harmonic notch filter bank

### Other Filters
- **DerivativeFilter** - Smooth derivative calculation
- **ModeFilter** - Statistical mode filter
- **AverageFilter** - Moving average filter
- **SlewLimiter** - Rate limiter with oscillation detection

## Usage Example

```cpp
#include "LowPassFilter.h"
#include "NotchFilter.h"

// Low pass filter example
LowPassFilterFloat lpf;
lpf.set_cutoff_frequency(20.0f);  // 20 Hz cutoff
float filtered = lpf.apply(raw_value, dt);

// Constant dt version (more efficient)
LowPassFilterConstDtFloat lpf_const;
lpf_const.set_cutoff_frequency(1000.0f, 20.0f);  // 1kHz sample rate, 20Hz cutoff
float filtered = lpf_const.apply(raw_value);

// Notch filter example
NotchFilterFloat notch;
notch.init(1000.0f, 50.0f, 10.0f, 20.0f);  // 1kHz sample, 50Hz center, 10Hz BW, 20dB atten
float filtered = notch.apply(raw_value);

// Harmonic notch filter example
HarmonicNotchFilterVector3f harmonic;
harmonic.allocate_filters(1, 0x07, 1);  // 1 source, harmonics 1+2+3, single notch
harmonic.init(1000.0f, 100.0f, 20.0f, 30.0f);  // Initialize
Vector3f filtered = harmonic.apply(gyro_data);
harmonic.update(motor_freq);  // Update center frequency dynamically
```

## Platform Integration

### Time Function
The library uses `filter_hal::millis()` for timing (used by SlewLimiter). You should provide your own implementation:

```cpp
// In your code, before including filter headers or in a .cpp file:
namespace filter_hal {
    uint32_t millis() {
        // Return milliseconds since boot/epoch
        return HAL_GetTick();  // Example for STM32
    }
}
```

The default implementation uses `std::chrono::steady_clock`.

### Vector Types
The library provides simple `Vector2f` and `Vector3f` classes in `filter_math.h`. You can also use your own vector types as long as they support basic arithmetic operations.

## Build with CMake

```bash
mkdir build && cd build
cmake ..
cmake --build .
```

Or integrate into your project:

```cmake
add_subdirectory(libs/ardupilot_filter_lib)
target_link_libraries(your_target PRIVATE ardupilot_filter)
```

## Configuration

Edit `filter_config.h` to adjust:
- `HNF_MAX_HARMONICS` - Maximum harmonics for harmonic notch (default: 16)
- `HNF_MAX_FILTERS` - Maximum number of notch filters (default: 12)

## License

GNU General Public License v3.0 (same as ArduPilot)

## Original Source

Based on ArduPilot filter library:
https://github.com/ArduPilot/ardupilot/tree/master/libraries/Filter

