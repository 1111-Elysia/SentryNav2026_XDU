// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

#pragma once

#include <src/embedded-filter-interface.h>
#include "core/extension.h"
#include <src/core/options-container.h>
#include <src/core/options-watcher.h>

namespace librealsense {
class temporal_embedded_filter
    : virtual public embedded_filter_interface
    , public options_container
{
public:
    virtual ~temporal_embedded_filter() = default;

    rsutils::subscription register_options_changed_callback(options_watcher::callback&&) override;

protected:
    options_watcher _options_watcher;
};

MAP_EXTENSION( RS2_EXTENSION_TEMPORAL_EMBEDDED_FILTER, librealsense::temporal_embedded_filter);

}  // namespace librealsense
