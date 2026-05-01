// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

#include <realdds/dds-embedded-filter.h>
#include "rs-dds-embedded-filter.h" 

using rsutils::json;

namespace librealsense {


    rs_dds_embedded_filter::rs_dds_embedded_filter(const std::shared_ptr< realdds::dds_embedded_filter >& dds_embedded_filter,
    set_embedded_filter_callback set_ef_cb,
    query_embedded_filter_callback query_ef_cb)
    : _dds_ef(dds_embedded_filter)
    , _set_ef_cb( set_ef_cb )
    , _query_ef_cb( query_ef_cb )
{
}

    rsutils::json rs_dds_embedded_filter::dds_option_to_name_and_value_json(std::shared_ptr<realdds::dds_option> option, const json& value)
    {
        json j = json::object();
        j[option->get_name()] = value;
        return j;
    }

}  // namespace librealsense
