/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-12-19     mengfanyuc   the first version
 */
#ifndef __JOYSTICK_H_
#define __JOYSTICK_H_

#define KEY_ON_TIME     10      //10ms
#define KEY_OFF_TIME    5       //5ms

#define LED_ON_TIME     20      //2min

enum e_key_state
{
    KEY_ON,
    KEY_OFF,
};

enum e_button_num
{
    BUTTON_UP,
    BUTTON_DOWN,
    BUTTON_LEFT,
    BUTTON_RIGHT,
    BUTTON_A,
    BUTTON_B,
    BUTTON_C,
    BUTTON_D,
    BUTTON_E,
    BUTTON_F,
    BUTTON_SELECT,
    BUTTON_START,

    BUTTON_NUM,
};

struct __key_state
{
    enum e_button_num button_num;
    enum e_key_state key_state;
};

struct __button_info
{
    void (*button_thread)(void *param);
    void (*button_irq_callback)(void *param);
    rt_base_t button_pin_num;
};

struct __key_info
{
    rt_uint8_t key_enable_cnt;
    rt_uint8_t key_disable_cnt;
};

extern void rt_button_thread_init(void *param);

#endif
