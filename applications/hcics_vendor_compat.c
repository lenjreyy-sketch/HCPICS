/*
 * Compatibility glue for vendor binary libraries.
 *
 * The AP6212/CYWL6208 WICED binary library references rt_assert_handler even
 * in release builds. RT-Thread only provides that symbol when RT_DEBUG is
 * enabled, so keep a release-safe fallback here.
 */

#include <rtthread.h>

#ifndef RT_DEBUG
void rt_assert_handler(const char *ex_string, const char *func, rt_size_t line)
{
    volatile char wait_forever = 0;

    rt_kprintf("(%s) vendor assertion failed at function:%s, line number:%d\n",
               ex_string, func, (int)line);

    while (wait_forever == 0)
    {
    }
}
#endif
