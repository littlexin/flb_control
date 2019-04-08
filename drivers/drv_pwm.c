/*
 * Change Logs:
 * Date           Author         Notes
 * 2019-04-6     zxin   			 first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>


#define TIM4_CNT_CLK    2100000
#define TIM4_PWM_CLK    21000


static uint16_t Prescaler;
static uint16_t Period;

static void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /* GPIOC clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    
    /* GPIOC Configuration: TIM4 CH1 (PD12)  , CH2 (PD13), CH3 (PD14), CH4 (PD15) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStructure); 

    /* Connect TIM4 pins to AF2 */  
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
}

static void TIM_Configuration(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef       TIM_OCInitStructure;
    
    /* TIM4 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    
    /* 
    In this example TIM4 input clock (TIM4CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1.   
    TIM4CLK = 2 * PCLK1  
    PCLK1 = HCLK / 4 
    => TIM3CLK = HCLK / 2 = SystemCoreClock /2

    To get TIM4 counter clock at 2 MHz, the prescaler is computed as follows:
    Prescaler = (TIM3CLK / TIM4 counter clock) - 1
    Prescaler = ((SystemCoreClock /2) /2 MHz) - 1 
    */
	
//    Prescaler = ((SystemCoreClock/2) / TIM4_CNT_CLK) - 1;
//    Period = TIM4_CNT_CLK / TIM4_PWM_CLK - 1;
    
		Prescaler = 839;
		Period = 1000;
		
    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = Period;
    TIM_TimeBaseStructure.TIM_Prescaler = Prescaler;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    
    /* PWM1 Mode configuration: Channel1 ~ Channel4 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);
    TIM_OC4Init(TIM4, &TIM_OCInitStructure);
    
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM4, ENABLE);
    
    /* TIM4 enable counter */
    TIM_Cmd(TIM4, ENABLE);
}

void pwm_set_duty_ratio(int channel, int ratio)
{
    ratio *= 0.01 * (Period + 1);
    
    switch (channel)	
    {
        case 1: TIM_SetCompare1(TIM4, ratio);break;
        case 2: TIM_SetCompare2(TIM4, ratio);break;
        case 3: TIM_SetCompare3(TIM4, ratio);break;
        case 4: TIM_SetCompare4(TIM4, ratio);break;
        default:
            break;
    }
}
void pwm_clear(void)
{
	TIM_SetCompare1(TIM4, 0);
	TIM_SetCompare2(TIM4, 0);
	TIM_SetCompare3(TIM4, 0);
	TIM_SetCompare4(TIM4, 0);
}

int pwm_init(void)
{
    GPIO_Configuration();
    TIM_Configuration();
//    rt_kprintf("ok\n");
    return 0;
}
INIT_BOARD_EXPORT(pwm_init);

#ifdef RT_USING_FINSH
#include <finsh.h>

void cmd_pwm_set_duty_ratio(int argc, char *argv[])
{
    if (argc >= 3)
    {
        int i;
        
        for (i = 1; i < argc; i++)
        {
            switch (argv[i++][1])
            {
                case '1': pwm_set_duty_ratio(1, atoi(argv[i]));break;
                case '2': pwm_set_duty_ratio(2, atoi(argv[i]));break;
                case '3': pwm_set_duty_ratio(3, atoi(argv[i]));break;
                case '4': pwm_set_duty_ratio(4, atoi(argv[i]));break;
                default:
                    break;
            }
        }
    }
    else
        rt_kprintf("Usage: pwm [-1 ratio] [-2 ratio] [-3 ratio] [-4 ratio]\n");
}
MSH_CMD_EXPORT_ALIAS(cmd_pwm_set_duty_ratio, pwm, set output duty ratio);
MSH_CMD_EXPORT_ALIAS(pwm_clear, cpwm, clear all pwm duty ratio);
#endif

