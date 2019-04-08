#ifndef _PID_H_
#define _PID_H_

#ifdef PID_DEBUG
#define pid_log(fmt, arg...)        rt_kprintf(fmt, ##arg)
#else
#define pid_log(fmt, arg...)        do {} while (0)
#endif

struct pid
{
    /* public */
    float set;
    float actual;
    float Kp, Ki, Kd;
    float omin, omax;
    float imin, imax;
    
    /* private */
    float err;
    float err_last;
    float err_next;
    float integral;
    float out_last;
};

typedef struct pid *pid_t;

pid_t pid_create(void);
void  pid_delete(pid_t pid);
void  pid_reset(pid_t pid);

void  pid_config(pid_t pid, float Kp, float Ki, float Kd);
void  pid_set_output_limit(pid_t pid, float umin, float umax);
//float pid_position_ctrl(pid_t pid, float set, float actual);
float pid_incremental_ctrl(pid_t pid, float set, float actual);

#endif  /* _PID_H_ */
