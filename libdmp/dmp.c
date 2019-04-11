#include <rtthread.h>
#include <rtdevice.h>
#include <math.h>
#include "dmp.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

static struct rt_semaphore sem;

struct rt_i2c_bus_device *i2c_bus = RT_NULL;
struct euler_angle el = {0};

static unsigned short inv_row_to_scaler(const signed char *row)
{
    unsigned short b;
	
    if (row[0] > 0)    	b = 0;
    else if (row[0] < 0)b = 4;
    else if (row[1] > 0)b = 1;
    else if (row[1] < 0)b = 5;
    else if (row[2] > 0)b = 2;
    else if (row[2] < 0)b = 6;
    else        		b = 7;// error
	
    return b;
}

static unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */
    scalar  = inv_row_to_scaler(mtx);
    scalar |= inv_row_to_scaler(mtx + 3) << 3;
    scalar |= inv_row_to_scaler(mtx + 6) << 6;
	
    return scalar;
}

static unsigned char run_self_test(void)
{
	int result;
	long gyro[3], accel[3];
    
	result = mpu_run_self_test(gyro, accel);
	if (result == 0x3) 
	{
		/* Test passed. We can trust the gyro data here, 
         * so let's push it down to the DMP.
		 */
		float sens;
		unsigned short accel_sens;
		mpu_get_gyro_sens(&sens);
		gyro[0] = (long)(gyro[0] * sens);
		gyro[1] = (long)(gyro[1] * sens);
		gyro[2] = (long)(gyro[2] * sens);
		dmp_set_gyro_bias(gyro);
		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		dmp_set_accel_bias(accel);
		return 0;
	}
    else
        return 1;
}


void dmp_thread_entry(void* parameter)
{
    struct euler_angle *el = parameter;
    
    while(1)
    {
        if (rt_sem_take(&sem, RT_WAITING_FOREVER) == RT_EOK)
        {
            int index;
            float q[4];
            long quat[4];
            short gyro[3], accel[3], sensors;
            unsigned long sensor_timestamp;
            unsigned char more;
            
            if (dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more) || 
                !(sensors & INV_WXYZ_QUAT))
                continue;
            
            /* q30 to float */
            for(index = 0; index < 4; index++)
                q[index] = quat[index] / 1073741824.f;
            /* Calculate the Pitch, Roll, Yaw */
            el->pitch = asinf(-2 * q[1] * q[3] + 2 * q[0] * q[2]) * 57.3f;
            el->roll  = atan2f(2 * q[2] * q[3] + 2 * q[0] * q[1], 
                              -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1) * 57.3f;
            el->yaw   = atan2f(2 * (q[1] * q[2] + q[0] * q[3]), 
                               q[0] * q[0] + q[1] * q[1] - q[2] * q[2]- q[3] * q[3]) * 57.3f;
        }
    }
}


static void dmp_thread_init(void)
{
    rt_thread_t tid;

    tid = rt_thread_create("dmp",
        dmp_thread_entry, &el,
        1024, RT_THREAD_PRIORITY_MAX - 2, 20);

    if (tid != RT_NULL)
        rt_thread_startup(tid);
}


static void dmp_notify_update(void)
{
    rt_sem_release(&sem);
}

int dmp_reg_int_cb(struct int_param_s *int_param)
{
    rt_sem_init(&sem, "dmpsem", 0, RT_IPC_FLAG_FIFO);
    extern void (*exti_hook)(void);
    exti_hook = dmp_notify_update;
    
    return 0;
}

void dmp_get_ms(unsigned long *cnt)
{
    rt_tick_t tick;
    
    tick = rt_tick_get();
	*cnt = tick * 1000 / RT_TICK_PER_SECOND;
}

void dmp_delay_ms(unsigned long ms)
{
	if (ms < 1000 / RT_TICK_PER_SECOND)
		rt_thread_delay(1);
	else
		rt_thread_delay(ms / (1000 / RT_TICK_PER_SECOND));
}

int dmp_i2c_write(unsigned char slave_addr, unsigned char reg_addr,
     unsigned char length, unsigned char const *data)
{
    rt_size_t ret;
    struct rt_i2c_msg msg[2];
    
    msg[0].addr  = slave_addr;
    msg[0].flags = RT_I2C_WR;
    msg[0].len   = 1;
    msg[0].buf   = (rt_uint8_t *)&reg_addr;
    msg[1].addr  = slave_addr;
    msg[1].flags = RT_I2C_WR | RT_I2C_NO_START;
    msg[1].len   = length;
    msg[1].buf   = (rt_uint8_t *)data;
    
    ret = rt_i2c_transfer(i2c_bus, msg, 2);
    
    return (ret == 2) ? 0 : -1;
}

int dmp_i2c_read(unsigned char slave_addr, unsigned char reg_addr,
     unsigned char length, unsigned char *data)
{
    rt_size_t ret;
    struct rt_i2c_msg msg[2];
    
    msg[0].addr  = slave_addr;
    msg[0].flags = RT_I2C_WR;
    msg[0].len   = 1;
    msg[0].buf   = (rt_uint8_t *)&reg_addr;
    msg[1].addr  = slave_addr;
    msg[1].flags = RT_I2C_RD;
    msg[1].len   = length;
    msg[1].buf   = (rt_uint8_t *)data;
    
    ret = rt_i2c_transfer(i2c_bus, msg, 2);
    
    return (ret == 2) ? 0 : -1;
}

void dmp_get_eulerangle(struct euler_angle *euler)
{
    if (euler)
    {
        rt_enter_critical();
        rt_memcpy(euler, &el, sizeof(struct euler_angle));
        rt_exit_critical();
    }
}

int dmp_sys_init(void)
{
    static struct int_param_s int_param;
    static signed char orientation[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
    
    i2c_bus = rt_i2c_bus_device_find("i2c1");
    RT_ASSERT(i2c_bus);
    
    mpu_init(&int_param);
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    /* Push both gyro and accel data into the FIFO. */
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(DEFAULT_MPU_HZ);
    dmp_load_motion_driver_firmware();
    dmp_set_orientation(inv_orientation_matrix_to_scalar(orientation));
    dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT     | DMP_FEATURE_TAP | 
                       DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | 
                       DMP_FEATURE_SEND_CAL_GYRO  | DMP_FEATURE_GYRO_CAL);
    dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    
    run_self_test();
		dmp_thread_init();
    mpu_set_dmp_state(1);
    
    return 0;
}

#ifdef RT_USING_FINSH
#include <finsh.h>

int dump_eulerangle(void)
{
    struct euler_angle el;
    
    dmp_get_eulerangle(&el);
    rt_kprintf("pitch: %4d.%01d¡ã   roll: %4d.%01d¡ã    yaw: %4d.%01d¡ã\n",
                (int)el.pitch, (unsigned int)(el.pitch * 10) % 10, 
                (int)el.roll,  (unsigned int)(el.roll  * 10) % 10, 
                (int)el.yaw,   (unsigned int)(el.yaw   * 10) % 10);
                
    return 0;
}
MSH_CMD_EXPORT_ALIAS(dump_eulerangle, euler, dump euler angle from libdmp);
//INIT_APP_EXPORT(dmp_thread_init);


#endif /* RT_USING_FINSH */
