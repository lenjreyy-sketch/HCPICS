/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * HCICS-PA keeps ART-Pi OTA/FAL support for Wi-Fi image verification, but the
 * YMODEM firmware update entry is disabled for the production vehicle build.
 */

#include <rtthread.h>

#ifdef ART_PI_USING_OTA_LIB
int rtota_update_entry_disabled(void)
{
    return 0;
}
#endif
