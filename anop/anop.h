#ifndef _ANO_PROTOCOL_H_
#define _ANO_PROTOCOL_H_

#include <rtthread.h>

#define ANOP_HEAD           (rt_uint8_t)0x88
#define ANOP_HLEN           (rt_uint8_t)0x03
#define ANOP_TLEN           (rt_uint8_t)0x01

#define ANOP_MAX_DATA_LEN   (rt_uint8_t)0x1C

#define ANOP_FUNC_CUSTOM_1  (rt_uint8_t)0xA1
#define ANOP_FUNC_CUSTOM_2  (rt_uint8_t)0xA2
#define ANOP_FUNC_CUSTOM_3  (rt_uint8_t)0xA3
#define ANOP_FUNC_CUSTOM_4  (rt_uint8_t)0xA4
#define ANOP_FUNC_CUSTOM_5  (rt_uint8_t)0xA5
#define ANOP_FUNC_CUSTOM_6  (rt_uint8_t)0xA6
#define ANOP_FUNC_CUSTOM_7  (rt_uint8_t)0xA7
#define ANOP_FUNC_CUSTOM_8  (rt_uint8_t)0xA8
#define ANOP_FUNC_CUSTOM_9  (rt_uint8_t)0xA9
#define ANOP_FUNC_CUSTOM_10 (rt_uint8_t)0xAA

#define ANOP_FUNC_FLY_CTRL  (rt_uint8_t)0xAF

__packed struct anop
{
    rt_uint8_t  head;
    rt_uint8_t  func;
    rt_uint8_t  len;
    rt_uint8_t* data;
    rt_uint8_t  checksum;
};

rt_err_t anop_upload_char (rt_uint8_t func, char  *data, rt_uint8_t len);
rt_err_t anop_upload_short(rt_uint8_t func, short *data, rt_uint8_t len);
rt_err_t anop_upload_long (rt_uint8_t func, long  *data, rt_uint8_t len);
rt_err_t anop_upload_float(rt_uint8_t func, float *data, rt_uint8_t len);

#endif  /* _ANO_PROTOCOL_H_ */
