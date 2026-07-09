/*
 * Compatibility symbols required by prebuilt ART-Pi Wi-Fi libraries.
 */

#include <rtthread.h>
#include <string.h>

#if defined(RT_KSERVICE_USING_STDLIB)
#undef rt_strcmp
rt_int32_t rt_strcmp(const char *cs, const char *ct)
{
    return (rt_int32_t)strcmp(cs, ct);
}
#endif
