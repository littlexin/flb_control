#include <rtthread.h>
#include <rtdevice.h>
#include <math.h>
#include "drv_pwm.h"
#include "pid.h"
#include "dmp.h"
#ifdef RT_USING_ANOP
#include "anop.h"
#endif

#define PENDULUM_CYCLE      1188        /* 1188 millisecond, rod length 0.35 meters */
#define MACHINE_HEIGTH      49          /* heigth of the machine */
#define PI                  3.14159f    /* ¦° value */
#define RADIAN_TO_ANGLE     180 / PI    /* convert radian to angle */

#define SAMPLE_INTERVAL     20          /* 100 millisecond */
#define OUTPUT_LIMIT        60          /* 60% duty ratio */

#define KP                  0
#define KI                  0
#define KD                  0

static pid_t pid_x;
static pid_t pid_y;
static struct rt_timer timer;
static struct rt_semaphore sem;

static void swing_move(int duty_ratio_x, int duty_ratio_y)
{
    /* direction x */
    if (duty_ratio_x >= 0)
    {
        /* forward */
        pwm_set_duty_ratio(1, 0);
        pwm_set_duty_ratio(2, duty_ratio_x);
    }
    else
    {
        /* backward */
        pwm_set_duty_ratio(1, -duty_ratio_x);
        pwm_set_duty_ratio(2, 0);
    }
    
    /* direction y */
    if (duty_ratio_y >= 0)
    {
        /* forward */
        pwm_set_duty_ratio(4, 0);
        pwm_set_duty_ratio(3, duty_ratio_y);
    }
    else
    {
        /* backward */
        pwm_set_duty_ratio(4, -duty_ratio_y);
        pwm_set_duty_ratio(3, 0);
    }
}

static void swing_mode_1(void)
{
    struct euler_angle el;
    float duty_ratio_x;
    float duty_ratio_y;
    float theta;
    float angle;
    float set_pitch;
    float set_roll;
    static rt_tick_t t = 0;
    
    while (1)
    {
        rt_sem_take(&sem, RT_WAITING_FOREVER);
        
        /* calculate angle amplitude */
        angle = atan(30.0f / MACHINE_HEIGTH) * RADIAN_TO_ANGLE;
        /* calculate current target radian */
        theta = t * (2 * PI / PENDULUM_CYCLE);
        /* calculate current target angle */
        set_pitch = angle * sin(theta);
        set_roll  = 0.0f;
        /* fetch currnet euler angle */
        dmp_get_eulerangle(&el);
        /* calculate the output duty ratio */
        duty_ratio_x = pid_incremental_ctrl(pid_x, set_pitch, el.pitch);
        duty_ratio_y = pid_incremental_ctrl(pid_y, set_roll, el.roll);
        swing_move(duty_ratio_x, duty_ratio_y);
        
        t += SAMPLE_INTERVAL;
     
    #ifdef RT_USING_ANOP
        anop_upload_float(ANOP_FUNC_CUSTOM_1, &set_pitch, 1);
        anop_upload_float(ANOP_FUNC_CUSTOM_2, &el.pitch, 1);
        anop_upload_float(ANOP_FUNC_CUSTOM_3, &duty_ratio_x, 1);
        
//        anop_upload_float(ANOP_FUNC_CUSTOM_6, &set_roll, 1);
//        anop_upload_float(ANOP_FUNC_CUSTOM_7, &el.roll, 1);
//        anop_upload_float(ANOP_FUNC_CUSTOM_8, &duty_ratio_y, 1);
    #endif
    }
}

static void swing_mode_2(void)
{
    while(1)
    {
        rt_sem_take(&sem, RT_WAITING_FOREVER);
    }
}

static void swing_mode_3(void)
{
    while(1)
    {
        rt_sem_take(&sem, RT_WAITING_FOREVER);
    }
}

static void swing_mode_4(void)
{
    while(1)
    {
        rt_sem_take(&sem, RT_WAITING_FOREVER);
    }
}

static void swing_mode_5(void)
{
    while(1)
    {
        rt_sem_take(&sem, RT_WAITING_FOREVER);
    }
}

static void swing_timer_cb(void *args)
{
    rt_sem_release(&sem);
}

static void swing_thread_entry(void *parameter)
{
    void (*swing_mode)(void) = (void(*)(void))parameter;
    
    /* execute the setting mode */
    swing_mode();
}

void swing_init(int mode)
{
    rt_thread_t tid;
    void (*_mode)(void);
    
    pid_x = pid_create();
    pid_y = pid_create();
    pid_set_output_limit(pid_x, -OUTPUT_LIMIT, OUTPUT_LIMIT);
    pid_set_output_limit(pid_y, -OUTPUT_LIMIT, OUTPUT_LIMIT);
    pid_config(pid_x, KP, KI, KD);
    pid_config(pid_y, KP, KI, KD);
    
    switch(mode)
    {
        case 1: _mode = swing_mode_1;break;
        case 2: _mode = swing_mode_2;break;
        case 3: _mode = swing_mode_3;break;
        case 4: _mode = swing_mode_4;break;
        case 5: _mode = swing_mode_5;break;
    }
    
    rt_sem_init(&sem, "swing_sem", 0, RT_IPC_FLAG_FIFO);
		
    tid = rt_thread_create("swing", swing_thread_entry, 
        _mode, 1024, RT_THREAD_PRIORITY_MAX/2, 10);
    if (tid != RT_NULL)
        rt_thread_startup(tid);
    rt_timer_init(&timer, "swing_timer", swing_timer_cb, RT_NULL, 
        rt_tick_from_millisecond(SAMPLE_INTERVAL), RT_TIMER_FLAG_PERIODIC);
    rt_timer_start(&timer);
    
    swing_move(0, 0);
}

void swing_deinit(void)
{
    rt_thread_t tid;
    
    rt_timer_stop(&timer);
    rt_timer_detach(&timer);
    tid = rt_thread_find("swing");
    if (tid != RT_NULL)
        rt_thread_delete(tid);
    rt_sem_detach(&sem);
    
    pid_reset(pid_x);
    pid_reset(pid_y);
    pid_delete(pid_x);
    pid_delete(pid_y);
    
    swing_move(0, 0);
}

#ifdef RT_USING_FINSH
#include <finsh.h>

void cmd_swing_pid(int argc, char *argv[])
{
    if (argc >= 3)
    {
        int i;
        float Kp = pid_x->Kp;
        float Ki = pid_x->Ki;
        float Kd = pid_x->Kd;

        for (i = 1; i < argc; i++)
        {
            switch (argv[i++][1])
            {
                case 'p': Kp = atof(argv[i]);break;
                case 'i': Ki = atof(argv[i]);break;
                case 'd': Kd = atof(argv[i]);break;
                default:
                    break;
            }
        }
        pid_config(pid_x, Kp, Ki, Kd);
        pid_config(pid_y, Kp, Ki, Kd);
    }
    else
        rt_kprintf("Usage: swpid [-p kp] [-i ki] [-d kd]\n");
}
MSH_CMD_EXPORT_ALIAS(cmd_swing_pid, swpid, config pid value for wind swing);

void cmd_swing_init(int argc, char *argv[])
{
    if (argc == 2)
        swing_init(atoi(argv[1]));
    else
        rt_kprintf("Usage: swinit <mode: 1~5>\n");
}
MSH_CMD_EXPORT_ALIAS(cmd_swing_init, swinit, initialize wind swing with mode);

void cmd_swing_deinit(int argc, char *argv[])
{
    swing_deinit();
}
MSH_CMD_EXPORT_ALIAS(cmd_swing_deinit, swdeinit, deinitialize wind swing);
#endif
