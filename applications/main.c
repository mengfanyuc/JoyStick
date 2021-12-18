/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-12-19     mengfanyuc   first version
 */

#include "board.h"
#include "joystick.h"

int main(void)
{
    rt_thread_t tid = rt_thread_create("JoyStick",
                        rt_button_thread_init, RT_NULL,
                        512, 8, 10);
    if (tid != RT_NULL)
        rt_thread_startup(tid);
    return RT_EOK;
}
