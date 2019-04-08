#include "anop.h"

static rt_device_t dev;

rt_inline short blendian_s(short x)
{
    return  ((x & 0x00FF) << 8 ) | 
            ((x & 0xFF00) >> 8);
}

rt_inline long blendian_l(long x)
{
    return  ((x & 0x000000FF) << 24) |  
            ((x & 0x0000FF00) << 8) |  
            ((x & 0x00FF0000) >> 8) | 
            ((x & 0xFF000000) >> 24);   
}

rt_inline float blendian_f(float x)
{
    union
    {
        float f;
        char c[4];
    }float1, float2;
    
    float1.f = x;
    float2.c[0] = float1.c[3];
    float2.c[1] = float1.c[2];
    float2.c[2] = float1.c[1];
    float2.c[3] = float1.c[0];

    return float2.f;
}

rt_err_t anop_upload_char(rt_uint8_t func, char *data, rt_uint8_t len)
{
    int i;
    struct anop frame;
    
    RT_ASSERT(data);
    
    rt_memset(&frame, 0, sizeof(struct anop));
    frame.head = ANOP_HEAD;
    frame.func = func;
    frame.len  = len * sizeof(char);
    if (frame.len > ANOP_MAX_DATA_LEN)
        return -RT_EFULL;
    frame.data = (rt_uint8_t *)data;
    frame.checksum += frame.head + frame.func + frame.len;
    for (i = 0; i < frame.len; i++)
        frame.checksum += frame.data[i];
    
    rt_device_write(dev, 0, &frame, ANOP_HLEN);
    rt_device_write(dev, 0, frame.data, frame.len);
    rt_device_write(dev, 0, &frame.checksum, ANOP_TLEN);
    
    return RT_EOK;
}

rt_err_t anop_upload_short(rt_uint8_t func, short *data, rt_uint8_t len)
{
    int i;
    struct anop frame;
    
    RT_ASSERT(data);
    
    rt_memset(&frame, 0, sizeof(struct anop));
    frame.head = ANOP_HEAD;
    frame.func = func;
    frame.len  = len * sizeof(short);
    if (frame.len > ANOP_MAX_DATA_LEN)
        return -RT_EFULL;
    for (i = 0; i < len; i++)
        data[i] = blendian_s(data[i]);
    frame.data = (rt_uint8_t *)data;
    frame.checksum += frame.head + frame.func + frame.len;
    for (i = 0; i < frame.len; i++)
        frame.checksum += frame.data[i];
    
    rt_device_write(dev, 0, &frame, ANOP_HLEN);
    rt_device_write(dev, 0, frame.data, frame.len);
    rt_device_write(dev, 0, &frame.checksum, ANOP_TLEN);
    
    return RT_EOK;
}

rt_err_t anop_upload_long(rt_uint8_t func, long *data, rt_uint8_t len)
{
    int i;
    struct anop frame;
    
    RT_ASSERT(data);
    
    rt_memset(&frame, 0, sizeof(struct anop));
    frame.head = ANOP_HEAD;
    frame.func = func;
    frame.len  = len * sizeof(long);
    if (frame.len > ANOP_MAX_DATA_LEN)
        return -RT_EFULL;
    for (i = 0; i < len; i++)
        data[i] = blendian_l(data[i]);
    frame.data = (rt_uint8_t *)data;
    frame.checksum += frame.head + frame.func + frame.len;
    for (i = 0; i < frame.len; i++)
        frame.checksum += frame.data[i];
		
    rt_device_write(dev, 0, &frame, ANOP_HLEN);
    rt_device_write(dev, 0, frame.data, frame.len);
    rt_device_write(dev, 0, &frame.checksum, ANOP_TLEN);
		
    return RT_EOK;
}

rt_err_t anop_upload_float(rt_uint8_t func, float *data, rt_uint8_t len)
{
    int i;
    struct anop frame;
    
    RT_ASSERT(data);
    
    rt_memset(&frame, 0, sizeof(struct anop));
    frame.head = ANOP_HEAD;
    frame.func = func;
    frame.len  = len * sizeof(float);
    if (frame.len > ANOP_MAX_DATA_LEN)
        return -RT_EFULL;
    for (i = 0; i < len; i++)
        data[i] = blendian_f(data[i]);
    frame.data = (rt_uint8_t *)data;
    frame.checksum += frame.head + frame.func + frame.len;
    for (i = 0; i < frame.len; i++)
        frame.checksum += frame.data[i];

    rt_device_write(dev, 0, &frame, ANOP_HLEN);
    rt_device_write(dev, 0, frame.data, frame.len);
    rt_device_write(dev, 0, &frame.checksum, ANOP_TLEN);
    
    return RT_EOK;
}

int anop_init(void)
{
    dev = rt_device_find("uart3");
		RT_ASSERT(dev != RT_NULL);
	
    if (dev == RT_NULL)
        return -RT_EEMPTY;
    
    return rt_device_open(dev, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STREAM);
}
INIT_COMPONENT_EXPORT(anop_init);
