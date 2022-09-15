#
# Laird Connectivity Power Component
#
# Copyright (c) 2020-2022 Laird Connectivity
#
# SPDX-License-Identifier: LicenseRef-LairdConnectivity-Clause
#

if((CONFIG_LCZ_POWER_TYPE_EXTERNAL) OR (CONFIG_LCZ_POWER_TYPE_DIRECT))
    # Add include and source files
    zephyr_include_directories(include)
    zephyr_sources_ifdef(CONFIG_LCZ_POWER_TYPE_EXTERNAL src/lcz_power.c)
    zephyr_sources_ifdef(CONFIG_LCZ_POWER_TYPE_DIRECT src/lcz_power_direct.c)

    # Add framework ID, msgcode and type files
    add_fwk_id_file(${CMAKE_CURRENT_SOURCE_DIR}/framework/lcz_power_ids.h)
    add_fwk_msgcode_file(${CMAKE_CURRENT_SOURCE_DIR}/framework/lcz_power_msgcodes.h)
    add_fwk_type_file(${CMAKE_CURRENT_SOURCE_DIR}/framework/lcz_power_types.h)
endif()

