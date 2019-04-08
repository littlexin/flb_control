#ifndef _DMP_H_
#define _DMP_H_

#define MPU6050
#define DEFAULT_MPU_HZ  			50

#define DMP_DEBUG
#ifdef DMP_DEBUG
#define dmp_log_i(fmt, arg...)      rt_kprintf(fmt, ##arg)
#define dmp_log_e(fmt, arg...)      rt_kprintf(fmt, ##arg)
#else
#define dmp_log_i(fmt, arg...)      do {} while (0)
#define dmp_log_e(fmt, arg...)      do {} while (0)
#endif

struct euler_angle
{
    float pitch;
    float roll;
    float yaw;
};

struct int_param_s;

int  dmp_reg_int_cb(struct int_param_s *int_param);
void dmp_get_ms(unsigned long *cnt);
void dmp_delay_ms(unsigned long ms);
int  dmp_i2c_write(unsigned char slave_addr, unsigned char reg_addr,
     unsigned char length, unsigned char const *data);
int  dmp_i2c_read(unsigned char slave_addr, unsigned char reg_addr,
     unsigned char length, unsigned char *data);

void dmp_get_eulerangle(struct euler_angle *euler);
int  dmp_sys_init(void);

#endif  /* _DMP_H_ */
