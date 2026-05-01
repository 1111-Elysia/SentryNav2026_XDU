// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

#include "temporal-embedded-filter.h"

rsutils::subscription librealsense::temporal_embedded_filter::register_options_changed_callback(options_watcher::callback&& cb)
{
    return _options_watcher.subscribe(std::move(cb));
}
