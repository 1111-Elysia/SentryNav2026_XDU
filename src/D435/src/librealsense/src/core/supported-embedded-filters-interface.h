// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

#pragma once

#include <src/embedded-filter-interface.h>
#include "extension.h"

#include <functional>
#include <vector>


namespace librealsense {


class supported_embedded_filters_interface
{
public:
    virtual ~supported_embedded_filters_interface() = default;

    virtual embedded_filters get_supported_embedded_filters() const = 0;};

MAP_EXTENSION( RS2_EXTENSION_SUPPORTED_EMBEDDED_FILTERS, librealsense::supported_embedded_filters_interface );


}  // namespace librealsense
