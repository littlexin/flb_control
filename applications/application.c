/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 * 2014-04-27     Bernard      make code cleanup. 
 */

#include <board.h>
#include <rtthread.h>
#include <rtdevice.h>

#ifdef RT_USING_LWIP
#include <lwip/sys.h>
#include <lwip/api.h>
#include <netif/ethernetif.h>
#include "stm32f4xx_eth.h"
#endif

#ifdef RT_USING_GDB
#include <gdb_stub.h>
#endif

#define TIMER "timer0"

static rt_err_t timer_timeout_cb(rt_device_t dev, rt_size_t size)
{
    rt_kprintf("enter hardware timer isr\n");

    return 0;
}

int hwtimer(void)
{
	rt_err_t err;
	rt_hwtimerval_t val;
	rt_device_t dev = RT_NULL;
	rt_tick_t tick;
	rt_hwtimer_mode_t mode;
	
	if((dev = rt_device_find(TIMER)) == RT_NULL)
	{
		rt_kprintf("No Device:%s\n", TIMER);
	}
//	else
//		rt_kprintf("ok\n");
	
	return 0;
}

void rt_init_thread_entry(void* parameter)
{
    /* GDB STUB */
#ifdef RT_USING_GDB
    gdb_set_device("uart6");
    gdb_start();
#endif
		//rt_components_init();
	
#ifdef RT_USING_I2C
    extern void rt_hw_i2c_init();
    
    rt_hw_i2c_init();
#endif
	
#ifdef RT_USING_DMP
    extern int dmp_sys_init(void);
    
    dmp_sys_init();
#endif
	
    /* LwIP Initialization */
#ifdef RT_USING_LWIP
    {
        extern void lwip_sys_init(void);

        /* register ethernetif device */
        eth_system_device_init();

        rt_hw_stm32_eth_init();

        /* init lwip system */
        lwip_sys_init();
        rt_kprintf("TCP/IP initialized!\n");
    }
#endif

#ifdef RT_USING_HWTIMER		
//		hwtimer();
#endif /*RT_USING_HWTIMER*/
		
}

int rt_application_init()
{
    rt_thread_t tid;

    tid = rt_thread_create("init",
        rt_init_thread_entry, RT_NULL,
        2048, RT_THREAD_PRIORITY_MAX/3, 20);

    if (tid != RT_NULL)
        rt_thread_startup(tid);
		

    return 0;
}

/*@}*/
