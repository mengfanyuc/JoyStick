/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-12-19     mengfanyuc   the first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"
#include "joystick.h"


static rt_sem_t sem_led;
static rt_mailbox_t mailbox_button;
static rt_sem_t sem_button[BUTTON_NUM];

#define LED0_PIN        GET_PIN(A,  6)

#define REGISTER_BUTTON_THREAD_NAME(button) thread_##button##_func
#define EXTERN_BUTTON_THREAD_FUNC(button)   extern void REGISTER_BUTTON_THREAD_NAME(button)(void *param);

#define REGISTER_BUTTON_IRQ_CALLBACK_NAME(button) irq_callback_##button##_func
#define EXTERN_BUTTON_IRQ_CALLBACK_FUNC(button)   extern void REGISTER_BUTTON_IRQ_CALLBACK_NAME(button)(void *param);

/*declarative button thread function*/
EXTERN_BUTTON_THREAD_FUNC(BUTTON_UP)
EXTERN_BUTTON_THREAD_FUNC(BUTTON_DOWN)
EXTERN_BUTTON_THREAD_FUNC(BUTTON_LEFT)
EXTERN_BUTTON_THREAD_FUNC(BUTTON_RIGHT)
EXTERN_BUTTON_THREAD_FUNC(BUTTON_A)
EXTERN_BUTTON_THREAD_FUNC(BUTTON_B)
EXTERN_BUTTON_THREAD_FUNC(BUTTON_C)
EXTERN_BUTTON_THREAD_FUNC(BUTTON_D)
EXTERN_BUTTON_THREAD_FUNC(BUTTON_E)
EXTERN_BUTTON_THREAD_FUNC(BUTTON_F)
EXTERN_BUTTON_THREAD_FUNC(BUTTON_SELECT)
EXTERN_BUTTON_THREAD_FUNC(BUTTON_START)

/*declarative button irq callback function*/
EXTERN_BUTTON_IRQ_CALLBACK_FUNC(BUTTON_UP)
EXTERN_BUTTON_IRQ_CALLBACK_FUNC(BUTTON_DOWN)
EXTERN_BUTTON_IRQ_CALLBACK_FUNC(BUTTON_LEFT)
EXTERN_BUTTON_IRQ_CALLBACK_FUNC(BUTTON_RIGHT)
EXTERN_BUTTON_IRQ_CALLBACK_FUNC(BUTTON_A)
EXTERN_BUTTON_IRQ_CALLBACK_FUNC(BUTTON_B)
EXTERN_BUTTON_IRQ_CALLBACK_FUNC(BUTTON_C)
EXTERN_BUTTON_IRQ_CALLBACK_FUNC(BUTTON_D)
EXTERN_BUTTON_IRQ_CALLBACK_FUNC(BUTTON_E)
EXTERN_BUTTON_IRQ_CALLBACK_FUNC(BUTTON_F)
EXTERN_BUTTON_IRQ_CALLBACK_FUNC(BUTTON_SELECT)
EXTERN_BUTTON_IRQ_CALLBACK_FUNC(BUTTON_START)

/*  button information configuration 
*   button thread name                           button irq callback function name                   pin number 
*/
const struct __button_info button_info_tab[BUTTON_NUM] =
{
    {REGISTER_BUTTON_THREAD_NAME(BUTTON_UP),     REGISTER_BUTTON_IRQ_CALLBACK_NAME(BUTTON_UP),       GET_PIN(B, 0),   },
    {REGISTER_BUTTON_THREAD_NAME(BUTTON_DOWN),   REGISTER_BUTTON_IRQ_CALLBACK_NAME(BUTTON_DOWN),     GET_PIN(B, 1),   },
    {REGISTER_BUTTON_THREAD_NAME(BUTTON_LEFT),   REGISTER_BUTTON_IRQ_CALLBACK_NAME(BUTTON_LEFT),     GET_PIN(B, 10),  },
    {REGISTER_BUTTON_THREAD_NAME(BUTTON_RIGHT),  REGISTER_BUTTON_IRQ_CALLBACK_NAME(BUTTON_RIGHT),    GET_PIN(B, 11),  },
    {REGISTER_BUTTON_THREAD_NAME(BUTTON_A),      REGISTER_BUTTON_IRQ_CALLBACK_NAME(BUTTON_A),        GET_PIN(A, 15),  },
    {REGISTER_BUTTON_THREAD_NAME(BUTTON_B),      REGISTER_BUTTON_IRQ_CALLBACK_NAME(BUTTON_B),        GET_PIN(B, 3),   },
    {REGISTER_BUTTON_THREAD_NAME(BUTTON_C),      REGISTER_BUTTON_IRQ_CALLBACK_NAME(BUTTON_C),        GET_PIN(B, 4),   },
    {REGISTER_BUTTON_THREAD_NAME(BUTTON_D),      REGISTER_BUTTON_IRQ_CALLBACK_NAME(BUTTON_D),        GET_PIN(B, 5),   },
    {REGISTER_BUTTON_THREAD_NAME(BUTTON_E),      REGISTER_BUTTON_IRQ_CALLBACK_NAME(BUTTON_E),        GET_PIN(B, 6),   },
    {REGISTER_BUTTON_THREAD_NAME(BUTTON_F),      REGISTER_BUTTON_IRQ_CALLBACK_NAME(BUTTON_F),        GET_PIN(B, 7),   },
    {REGISTER_BUTTON_THREAD_NAME(BUTTON_SELECT), REGISTER_BUTTON_IRQ_CALLBACK_NAME(BUTTON_SELECT),   GET_PIN(B, 8),   },
    {REGISTER_BUTTON_THREAD_NAME(BUTTON_START),  REGISTER_BUTTON_IRQ_CALLBACK_NAME(BUTTON_START),    GET_PIN(B, 9),   },
};

/**
 * implementation button thread function
 * wait until an io state change triggers an interrupt
 * after get a semaphore, the information will be processed and sent a mailbox to usb thread          
 * @param param:button_num.
 *
 * @return None.
 */
#define REGISTER_BUTTON_THREAD_FUNC(button)                                                         \
void REGISTER_BUTTON_THREAD_NAME(button)(void *param)                                               \
{                                                                                                   \
    rt_uint8_t *num = (rt_uint8_t *)param;;                                                         \
    struct __key_state *key_state;                                                                  \
    static struct __key_info key_info[BUTTON_NUM];                                                  \
                                                                                                    \
    while(1)                                                                                        \
    {                                                                                               \
        rt_err_t result = rt_sem_take(sem_button[*num], RT_WAITING_FOREVER);                        \
        if(result == RT_EOK)                                                                        \
        {                                                                                           \
            while(rt_pin_read(button_info_tab[*num].button_pin_num) == PIN_LOW)                     \
            {                                                                                       \
                key_info[*num].key_disable_cnt = 0;                                                 \
                if(key_info[*num].key_enable_cnt < KEY_ON_TIME)                                     \
                    key_info[*num].key_enable_cnt++;                                                \
                else                                                                                \
                {                                                                                   \
                    key_state = (struct __key_state *)rt_malloc(sizeof(struct __key_state));        \
                    RT_ASSERT(key_state != RT_NULL)                                                 \
                    key_state->button_num = *num;                                                   \
                    key_state->key_state = KEY_ON;                                                  \
                    if(rt_mb_send(mailbox_button, (rt_ubase_t)key_state) != RT_EOK)                 \
                        rt_free(key_state);                                                         \
                    break;                                                                          \
                }                                                                                   \
                rt_thread_mdelay(1);                                                                \
            }                                                                                       \
                                                                                                    \
            while(rt_pin_read(button_info_tab[*num].button_pin_num) == PIN_HIGH)                    \
            {                                                                                       \
                key_info[*num].key_enable_cnt = 0;                                                  \
                if(key_info[*num].key_disable_cnt < KEY_OFF_TIME)                                   \
                    key_info[*num].key_disable_cnt++;                                               \
                else                                                                                \
                {                                                                                   \
                    key_state = (struct __key_state *)rt_malloc(sizeof(struct __key_state));        \
                    RT_ASSERT(key_state != RT_NULL)                                                 \
                    key_state->button_num = *num;                                                   \
                    key_state->key_state = KEY_OFF;                                                 \
                    if(rt_mb_send(mailbox_button, (rt_ubase_t)key_state) != RT_EOK)                 \
                        rt_free(key_state);                                                         \
                    break;                                                                          \
                }                                                                                   \
                rt_thread_mdelay(1);                                                                \
            }                                                                                       \
        }                                                                                           \
    }                                                                                               \
}                                                                                                   \

/*register BUTTON_NUM button thread function*/
REGISTER_BUTTON_THREAD_FUNC(BUTTON_UP)
REGISTER_BUTTON_THREAD_FUNC(BUTTON_DOWN)
REGISTER_BUTTON_THREAD_FUNC(BUTTON_LEFT)
REGISTER_BUTTON_THREAD_FUNC(BUTTON_RIGHT)
REGISTER_BUTTON_THREAD_FUNC(BUTTON_A)
REGISTER_BUTTON_THREAD_FUNC(BUTTON_B)
REGISTER_BUTTON_THREAD_FUNC(BUTTON_C)
REGISTER_BUTTON_THREAD_FUNC(BUTTON_D)
REGISTER_BUTTON_THREAD_FUNC(BUTTON_E)
REGISTER_BUTTON_THREAD_FUNC(BUTTON_F)
REGISTER_BUTTON_THREAD_FUNC(BUTTON_SELECT)
REGISTER_BUTTON_THREAD_FUNC(BUTTON_START)

/**
 * io irq callback fuction
 * triggered when io status changed and send semaphore to button thread
 *            
 * @param param:button_num.
 *
 * @return None.
 */
#define REGISTER_BUTTON_IRQ_CALLBACK_FUNC(button)                                                   \
void REGISTER_BUTTON_IRQ_CALLBACK_NAME(button)(void *param)                                         \
{                                                                                                   \
    rt_uint8_t *num = (rt_uint8_t *)param;                                                          \
    rt_sem_release(sem_button[*num]);                                                               \
}                                                                                                   \

/*register BUTTON_NUM button irq callback function*/
REGISTER_BUTTON_IRQ_CALLBACK_FUNC(BUTTON_UP)
REGISTER_BUTTON_IRQ_CALLBACK_FUNC(BUTTON_DOWN)
REGISTER_BUTTON_IRQ_CALLBACK_FUNC(BUTTON_LEFT)
REGISTER_BUTTON_IRQ_CALLBACK_FUNC(BUTTON_RIGHT)
REGISTER_BUTTON_IRQ_CALLBACK_FUNC(BUTTON_A)
REGISTER_BUTTON_IRQ_CALLBACK_FUNC(BUTTON_B)
REGISTER_BUTTON_IRQ_CALLBACK_FUNC(BUTTON_C)
REGISTER_BUTTON_IRQ_CALLBACK_FUNC(BUTTON_D)
REGISTER_BUTTON_IRQ_CALLBACK_FUNC(BUTTON_E)
REGISTER_BUTTON_IRQ_CALLBACK_FUNC(BUTTON_F)
REGISTER_BUTTON_IRQ_CALLBACK_FUNC(BUTTON_SELECT)
REGISTER_BUTTON_IRQ_CALLBACK_FUNC(BUTTON_START)

/**
 * This function will send button state to USB
 * Send button state when button thread send a state changed message to mailbox
 * 
 * USB message definition(report descriptor configuration in hid.c):
 * -----------------------------------------------------------
 * |    byte 0    |     byte1    |          byte3            |
 * -----------------------------------------------------------
 * | x-coordinate | y-coordinate | button A-F | select/start |  
 * -----------------------------------------------------------
 *
 * Schematic diagram of direction key
 *    0 --------- 2 x-axis      ------------------------
 *      |       |               |  button | coordinate | 
 *      |   *   |               ------------------------
 *      |       |               |    up   |    (1,0)   |
 *    2 ---------               ------------------------
 * y-axis                       |   down  |    (1,2)   |
 *                              ------------------------
 *                              |   left  |    (0,1)   |
 *                              ------------------------
 *                              |  right  |    (2,1)   |
 *                              ------------------------
 *                              | default |    (1,1)   |
 *                              ------------------------              
 * 
 * @param None.
 *
 * @return None.
 */
void rt_usb_send_thread(void *param)
{
    struct __key_state *key_state;
    rt_int8_t x_axis = 0, y_axis = 0;
    static rt_uint8_t usb_send_data[3] = {0, 0, 0}; 

    rt_device_t hidd = rt_device_find("hidd");
    RT_ASSERT(hidd != RT_EOK)

    rt_err_t RT_UNUSED error = rt_device_open(hidd,RT_DEVICE_OFLAG_WRONLY);
    RT_ASSERT(error == RT_EOK)

    while(1)
    {
        if(rt_mb_recv(mailbox_button, (rt_ubase_t *)&key_state, RT_WAITING_FOREVER) == RT_EOK)
        {    
            switch(key_state->button_num)
            {
                case BUTTON_UP:
                    if(key_state->key_state == KEY_ON)
                        y_axis = -1;
                    else
                        y_axis = 0;     
                    break;
                case BUTTON_DOWN:
                    if(key_state->key_state == KEY_ON)
                        y_axis = 1;
                    else
                        y_axis = 0;     
                    break;
                case BUTTON_LEFT:
                    if(key_state->key_state == KEY_ON)
                        x_axis = -1;
                    else
                        x_axis = 0;
                    break;
                case BUTTON_RIGHT:
                    if(key_state->key_state == KEY_ON)
                        x_axis = 1;
                    else
                        x_axis = 0;
                    break;
                case BUTTON_A:
                case BUTTON_B:
                case BUTTON_C:
                case BUTTON_D:
                case BUTTON_E:
                case BUTTON_F:
                case BUTTON_SELECT:
                case BUTTON_START:
                    if(key_state->key_state == KEY_ON)
                        usb_send_data[2] |= 1<<(key_state->button_num-BUTTON_A);     
                    else if(key_state->key_state == KEY_OFF)
                        usb_send_data[2] &= ~(1<<(key_state->button_num-BUTTON_A));
                    break;
                default:
                    break;    
            }
            
            if(key_state->key_state == KEY_ON)
                rt_sem_release(sem_led);

            usb_send_data[0] = (rt_uint8_t)(x_axis+1);
            usb_send_data[1] = (rt_uint8_t)(y_axis+1);
            rt_device_write(hidd, HID_REPORT_ID_JOYSTICK, usb_send_data, 3);
            rt_free(key_state);      
        }
    }
}

/**
 * This function will light on/off led
 * light on led LED_ON_TIME*100ms when USB send a button on message
 * The count is cleared each time a button on message message is sent
 *  
 * @param None.
 *
 * @return None.
 */
void rt_led_thread(void *param)
{
    static rt_uint8_t led_cnt = LED_ON_TIME;

    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
    
    while(1)
    {
        if(led_cnt >= LED_ON_TIME)    
        {
            rt_pin_write(LED0_PIN, PIN_LOW);
            rt_err_t result = rt_sem_take(sem_led, RT_WAITING_FOREVER);
            if(result == RT_EOK)
                led_cnt = 0;
        }    
        else
        {
            led_cnt++;
            rt_pin_write(LED0_PIN, PIN_HIGH);
            if(rt_sem_take(sem_led, RT_WAITING_NO) == RT_EOK)
            {
                led_cnt = 0;
                (void)rt_sem_control(sem_led, RT_IPC_CMD_RESET, (void*)0);    
            }
            rt_thread_mdelay(100);
        }
    } 
}

/**
 * This function will create joystick resource, 
 * It will create a semaphore which send message to clear led on time when send a button on message to usb.
 * a mailbox which send io state message to usb thread
 * a thread which send processing IO status information to USB
 * a thread which led on/off
 * BUTTON_NUM pin irq callback function which get button information
 * BUTTON_NUM semaphore which send pin state change to button thread when interrupt trigger
 * BUTTON_NUM thread which process interrupt information to key state
 *  
 * @param None.
 *
 * @return None.
 */
void rt_button_thread_init(void *param)
{
    rt_thread_t tid[BUTTON_NUM];

    /*create led sem to transfer button information to led process*/
    sem_led = rt_sem_create("sem_led", 0, RT_IPC_FLAG_FIFO);
    RT_ASSERT(sem_led != RT_NULL)

    /*create a mailbox to transfer key information to USB*/
    mailbox_button = rt_mb_create("mailbox_button", BUTTON_NUM*5, RT_IPC_FLAG_FIFO);                  
    RT_ASSERT(mailbox_button != RT_NULL)

    /*create a usb send thread*/
    rt_thread_t usb_tid = rt_thread_create("usb_send",
                    rt_usb_send_thread, RT_NULL,
                    256, 3, 10);

    if(usb_tid != RT_NULL)
        rt_thread_startup(usb_tid);

    /*create a led thread*/
    rt_thread_t led_tid = rt_thread_create("led",
                    rt_led_thread, RT_NULL,
                    256, 10, 10);

    if(led_tid != RT_NULL)
        rt_thread_startup(led_tid);

    /*button_num do not free in order to transfer point address as other thread`s param*/
    rt_uint8_t *button_num = (rt_uint8_t *)rt_malloc(BUTTON_NUM*(sizeof(rt_uint8_t)));
    RT_ASSERT(button_num != RT_NULL)   

    for(rt_uint8_t i = 0; i < BUTTON_NUM; i++)
    {
        *button_num = i;
        /*set io input irq*/
        rt_pin_mode(button_info_tab[i].button_pin_num, PIN_MODE_INPUT);
        rt_pin_attach_irq(button_info_tab[i].button_pin_num, PIN_IRQ_MODE_RISING_FALLING,
                    button_info_tab[i].button_irq_callback, button_num);
        rt_pin_irq_enable(button_info_tab[i].button_pin_num, PIN_IRQ_ENABLE);

        /*create BUTTON_NUM sem to transfer irq information to key process*/
        sem_button[i] = rt_sem_create("sem_button", 0, RT_IPC_FLAG_FIFO);
        RT_ASSERT(sem_button[i] != RT_NULL) 

        /*create BUTTON_NUM thread to process key information*/
        tid[i] = rt_thread_create("button",
                    button_info_tab[i].button_thread, button_num,
                    256, 5, 10);

        if(tid[i] != RT_NULL)
           rt_thread_startup(tid[i]);   
        
        button_num++;    
    }
}
