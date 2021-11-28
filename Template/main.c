/*!
    \file  main.c
    \brief USART transmit and receive interrupt
*/

/*
    Copyright (C) 2017 GigaDevice

    2014-12-26, V1.0.0, platform GD32F1x0(x=3,5)
    2016-01-15, V2.0.0, platform GD32F1x0(x=3,5,7,9)
    2016-04-30, V3.0.0, firmware update for GD32F1x0(x=3,5,7,9)
    2017-06-19, V3.1.0, firmware update for GD32F1x0(x=3,5,7,9)
*/
#include "math.h"
#include "gd32f1x0.h"
#include <stdio.h>
#include "gd32f1x0_eval.h"
#include "gd32f1x0.h"
#include "systick.h"
#include "stdlib.h"

uint8_t Road_change_flag;
uint8_t check_flag;
uint8_t rece_data_count;
uint8_t rece_data_flag = RESET;
__IO uint8_t txcount = 0;
__IO uint16_t rxcount = 0;

int i, j, k;

//与传感器相关的变量
const int Nums_Getdata = 20;   //总计数次数
const double Expect_Pf = 0.99; //期望调整到的功率因子
// const double Capacitive_reactive_1uF = 3.14159 * 2 * 220 * 220 * 50 * 0.000001 / 1000; //1uF电容的无功补偿功率 0.0151976Kvar
const double Capacitive_reactive_1uF = 3.14159 * 2 * 220 * 220 * 50 * 0.000001;                                         //1uF电容的无功补偿功率 0.0151976var
const int Capacitive_reactive_C[3] = {10, 20, 40};                                                                      //3个补偿电容 单位uF
const unsigned char Read_ID = 0x01;                                                                                     //传感器编号 默认为0x01
unsigned char Tx_Buffer[8];                                                                                             //向传感器发送数据
unsigned char RX_Buffer[37];                                                                                            //接受传感器传回的原始数据 无符号16进制数
double Voltage_data, Current_data, Power_data;                                                                          //传感器返回的各种数据 处理后的十进制数
double Energy_data, Reactive_data, Pf_data, CO2_data, Temperature_data, Fre_data;                                       //传感器返回的各种数据 处理后的十进制数
double Power_data_Buffer[Nums_Getdata], Pf_data_buffer[Nums_Getdata];                                                   //保存传感器返回的正常范围内功率和功率因子
const int Cpa_set_arr[8][3] = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}, {0, 0, 1}, {1, 0, 1}, {0, 1, 1}, {1, 1, 1}}; //电容切换的8种方案
double Calculation_Capacity_plan[8];                                                                                    //保存每个方案的无功补偿容量
unsigned int SysNums_Getdata = 0;                                                                                       //读数次数累加器 到读数总次数后归0                                                                                   //读数间隔时间，单位毫秒                                                                                   //读数总次数
const unsigned int Danger_Voltage = 260;                                                                                //阈值电压  单位V
const unsigned int Danger_Current = 20;                                                                                 //阈值电流 单位A
unsigned long num = 0;                                                                                                  //这两个数据只能定义为该类型数据 num:读数总次数
double Previous_Q = 0, Current_Q = 0;                                                                                   //上一次投切的电容容量 这一次需要投切的电容容量
int ad_value_low = 1700;                                                                                                //ad采样投切阈值，最低值
int ad_value_high = 1900;                                                                                               //ad采样投切阈值，最高值

int feed_dog = 10; //狗碗，每次喂狗值会减一，需要在主函数中，不停刷新该值

void led_config(void);                                   //led初始化
void led_flash(int times);                               //点亮熄灭一次led 1，2，3 时间为250ms
void led_3_flash(int times, int delay_time);             //点亮熄灭一次led 1 时间为手动设置
void led_2_flash(int times, int delay_time);             //点亮熄灭一次led 2 时间为手动设置
void led_1_flash(int times, int delay_time);             //点亮熄灭一次led 3 时间为手动设置
void gpio_config(void);                                  //引脚初始化
void timer_config(void);                                 //定时器初始化
void usart_config(void);                                 //串口初始化
void adc_config(void);                                   //adc初始化
void wdgt_config(void);                                  //看门狗初始化
unsigned short adc0_transfer(unsigned char adc_channel); //返回选择的AD通道的采样结果，

void Open_road(int channel);     //投切某一组电容
void Close_road(int channel);    //移除某一组电容
void Road_dc_reset(void);        //复位所有电容通路，即断开所有通路
void Road_dc_close(int channel); //关闭继电器供电，在投切，移除，复位电容通路一段时间后需要调用这个函数

unsigned int calccrc(unsigned char crcbuf, unsigned int crc); //生成数据校验码
unsigned int chkcrc(unsigned char *buf, unsigned char len);   //检测数据校验码是否正确
void Get_Im1281B_data(void);                                  //向传感器发送取数据指令，读取返回的传感器数据
int Check_Im1281B_data(void);                                 //校验传感器的数据是否正确
int SaveData(void);                                           //通过传感器的数据判断电路状态：过压，过流，过补偿都会立刻切断所有电容 只有功率因子在0.5-1时，才会保存数据，进一步处理
void Calculation_Capacity(void);                              //计算每一个方案所能补偿的无功功率
void Cal_Cpa_plan(void);                                      //根据存储的数据，求取平均功率因子和平均功率，计算补偿电电容，选择方案进行投切
void road_ad_value(int fire, int cal);                        //返回投切时，该路的火线以及电容的电压和ad采样值

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    systick_config();

    timer_config();

    usart_config();

    adc_config();

    gpio_config();

    led_config();

    wdgt_config();

    Calculation_Capacity();

    delay_1ms(100);

    led_flash(1);

    //继电器复位
    Road_dc_reset();

    led_flash(1);

    printf("\nSYS start!\n");

    while (1)
    {

        feed_dog = 10;
        Get_Im1281B_data();
        if (SET == Check_Im1281B_data())
        {
            if (SET == SaveData())
            {
                SysNums_Getdata++;
                if (SysNums_Getdata == Nums_Getdata)
                {
                    led_flash(1);
                    SysNums_Getdata = 0;
                    Cal_Cpa_plan();
                }
            }
        }
        led_3_flash(1, 125);
        // delay_1ms(100);
    };
}

/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(EVAL_COM1, (uint8_t)ch);
    while (RESET == usart_flag_get(EVAL_COM1, USART_FLAG_TBE))
        ;
    return ch;
}
//看门狗初始化 计时时间1s
void wdgt_config(void)
{
    rcu_osci_on(RCU_IRC40K);
    /* wait till IRC40K is ready */
    while (SUCCESS != rcu_osci_stab_wait(RCU_IRC40K))
    {
    }
    fwdgt_config(625, FWDGT_PSC_DIV64);
    fwdgt_enable();
}
//控制板led
void led_flash(int times)
{
    int i;
    for (i = 0; i < times; i++)
    {
        /* delay 250 ms */
        delay_1ms(250);
        /* toggle the LED */
        gpio_bit_write(GPIOB, GPIO_PIN_3, (bit_status)((1 - gpio_output_bit_get(GPIOB, GPIO_PIN_3))));
        gpio_bit_write(GPIOA, GPIO_PIN_11, (bit_status)((1 - gpio_output_bit_get(GPIOA, GPIO_PIN_11))));
        gpio_bit_write(GPIOA, GPIO_PIN_12, (bit_status)((1 - gpio_output_bit_get(GPIOA, GPIO_PIN_12))));

        /* delay 250 ms */
        delay_1ms(250);
        /* toggle the LED */
        gpio_bit_write(GPIOB, GPIO_PIN_3, (bit_status)((1 - gpio_output_bit_get(GPIOB, GPIO_PIN_3))));
        gpio_bit_write(GPIOA, GPIO_PIN_11, (bit_status)((1 - gpio_output_bit_get(GPIOA, GPIO_PIN_11))));
        gpio_bit_write(GPIOA, GPIO_PIN_12, (bit_status)((1 - gpio_output_bit_get(GPIOA, GPIO_PIN_12))));
    }
}

void led_3_flash(int times, int delay_time)
{
    int i;
    for (i = 0; i < times; i++)
    {
        // PA11 LED3
        gpio_bit_write(GPIOA, GPIO_PIN_11, (bit_status)((1 - gpio_output_bit_get(GPIOA, GPIO_PIN_11))));
        delay_1ms(delay_time);
        gpio_bit_write(GPIOA, GPIO_PIN_11, (bit_status)((1 - gpio_output_bit_get(GPIOA, GPIO_PIN_11))));
    }
}
void led_2_flash(int times, int delay_time)
{
    int i;
    for (i = 0; i < times; i++)
    {
        //  PA12 LED2
        gpio_bit_write(GPIOA, GPIO_PIN_12, (bit_status)((1 - gpio_output_bit_get(GPIOA, GPIO_PIN_12))));
        delay_1ms(delay_time);
        gpio_bit_write(GPIOA, GPIO_PIN_12, (bit_status)((1 - gpio_output_bit_get(GPIOA, GPIO_PIN_12))));
    }
}
void led_1_flash(int times, int delay_time)
{
    int i;
    for (i = 0; i < times; i++)
    {
        // PB3  LED1
        gpio_bit_write(GPIOB, GPIO_PIN_3, (bit_status)((1 - gpio_output_bit_get(GPIOB, GPIO_PIN_3))));
        delay_1ms(delay_time);
        gpio_bit_write(GPIOB, GPIO_PIN_3, (bit_status)((1 - gpio_output_bit_get(GPIOB, GPIO_PIN_3))));
    }
}
/*! 
    \brief      configure the LEDs
    \param[in]  none
    \param[out] none
    \retval     none
  */
void led_config(void)
{
    gd_eval_led_init(LED1);
    gd_eval_led_init(LED2);
}

/*! 
    \brief      configure the GPIO ports
    \param[in]  none
    \param[out] none
    \retval     none
  */
void gpio_config(void)
{
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);

    gpio_mode_set(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
    gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_7);
    // gpio_mode_set(RCU_GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);

    //PB11-PB15,PB0,PB1 OUTPUT SET
    gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9);
    //  gpio_bit_reset(GPIOB, GPIO_PIN_1 | GPIO_PIN_0 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);

    //PA11 ,PA12 OUTPUT SET
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_11 | GPIO_PIN_12);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_11 | GPIO_PIN_12);
    // gpio_bit_reset(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);
}

/*!
    \brief      configure the TIMER peripheral
    \param[in]  none
    \param[out] none
    \retval     none
  */
void timer_config(void)
{
    /* -----------------------------------------------------------------------
    TIMER2CLK is 100KHz
    TIMER2 channel0 duty cycle = (25000/ 50000)* 100  = 50%
    ----------------------------------------------------------------------- */
    // timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER2);

    timer_deinit(TIMER2);

    /* TIMER configuration */
    // timer_initpara.prescaler         = 719;
    timer_initpara.prescaler = 719;
    timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
    timer_initpara.period = 49999;
    timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER2, &timer_initpara);

    timer_interrupt_enable(TIMER2, TIMER_INT_UP);
    timer_enable(TIMER2);
    nvic_irq_enable(TIMER2_IRQn, 1, 1);
}

void usart_config(void)
{
    /* USART interrupt configuration */
    nvic_irq_enable(USART0_IRQn, 0, 0);
    gd_eval_com_init(EVAL_COM1);
}

/*!
    \brief      configure the adc peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void adc_config(void)
{

    // /* ADC software trigger enable */
    // adc_software_trigger_enable(ADC_INSERTED_CHANNEL);
    rcu_periph_clock_enable(RCU_GPIOA);
    /* enable GPIOB clock */
    rcu_periph_clock_enable(RCU_GPIOB);
    /* enable ADC0 clock */
    rcu_periph_clock_enable(RCU_ADC);
    /* config ADC clock */
    rcu_adc_clock_config(RCU_ADCCK_APB2_DIV4);
    adc_deinit();
    adc_special_function_config(ADC_SCAN_MODE, ENABLE);
    /* ADC data alignment config */
    adc_data_alignment_config(ADC_DATAALIGN_RIGHT);
    adc_special_function_config(ADC_CONTINUOUS_MODE, DISABLE);
    /* ADC channel length config */
    adc_channel_length_config(ADC_REGULAR_CHANNEL, 1);
    /* ADC trigger config */
    adc_external_trigger_source_config(ADC_REGULAR_CHANNEL, ADC_EXTTRIG_REGULAR_SWRCST);
    adc_external_trigger_config(ADC_REGULAR_CHANNEL, ENABLE);
    delay_1ms(1);
    /* enable ADC interface */
    adc_enable();
    /* ADC calibration and reset calibration */
    adc_calibration_enable();
    // adc_software_trigger_enable(ADC_REGULAR_CHANNEL);
}

unsigned short adc0_transfer(unsigned char adc_channel)
{
    uint16_t ADC_temp;
    /* ADC regular channel config */
    adc_regular_channel_config(0, adc_channel, ADC_SAMPLETIME_239POINT5);
    adc_software_trigger_enable(ADC_REGULAR_CHANNEL);
    while (1)
    {
        if (adc_flag_get(ADC_FLAG_EOC) != RESET)
            break;
    }
    ADC_temp = adc_regular_data_read();
    adc_flag_clear(ADC_FLAG_EOC);
    return ADC_temp;
}

//OPEN ACCESS
void Open_road(int channel)
{
    int temp_ad_Fire = 0;
    int temp_ad_ch = 0;
    int count = 0;
    Road_change_flag = SET;
    if (channel == 1)
    {
        while (1)
        {
            feed_dog = 10;
            temp_ad_Fire = adc0_transfer(ADC_CHANNEL_1);
            // delay_1ms(10);
            temp_ad_ch = adc0_transfer(ADC_CHANNEL_2);
            if (ad_value_low < temp_ad_Fire && ad_value_high > temp_ad_Fire)
            {
                if (ad_value_low < temp_ad_ch && ad_value_high > temp_ad_ch)
                {
                    if (abs(temp_ad_ch - temp_ad_Fire) < 40)
                    {
                        GPIO_BOP(GPIOB) = GPIO_PIN_3; //通路1开启指示灯亮
                        //open road 1
                        //output "01"
                        GPIO_BC(GPIOB) = GPIO_PIN_8;
                        GPIO_BOP(GPIOB) = GPIO_PIN_9;
                        road_ad_value(temp_ad_Fire, temp_ad_ch);
                        break;
                    }
                }
            }
            // road_ad_value(temp_ad_Fire, temp_ad_ch);
            // led_1_flash(1, 10);
            count++;
            if (count == 500)
            {
                count = 0;
                break;
            }
        }
    }
    if (channel == 2)
    {
        while (1)
        {
            feed_dog = 10;
            temp_ad_Fire = adc0_transfer(ADC_CHANNEL_1);
            temp_ad_ch = adc0_transfer(ADC_CHANNEL_3);
            if (ad_value_low < temp_ad_Fire && ad_value_high > temp_ad_Fire)
            {
                if (ad_value_low < temp_ad_ch && ad_value_high > temp_ad_ch)
                {
                    if (abs(temp_ad_ch - temp_ad_Fire) < 40)
                    {
                        GPIO_BOP(GPIOA) = GPIO_PIN_12; //通路1开启指示灯亮
                        //open road 2
                        //output "01"
                        GPIO_BC(GPIOB) = GPIO_PIN_6;
                        GPIO_BOP(GPIOB) = GPIO_PIN_7;
                        road_ad_value(temp_ad_Fire, temp_ad_ch);
                        break;
                    }
                }
            }
            // road_ad_value(temp_ad_Fire, temp_ad_ch);
            // led_2_flash(1, 10);
            count++;
            if (count == 500)
            {
                count = 0;
                break;
            }
        }
    }
    if (channel == 3)
    {
        while (1)
        {
            feed_dog = 10;
            temp_ad_Fire = adc0_transfer(ADC_CHANNEL_1);
            temp_ad_ch = adc0_transfer(ADC_CHANNEL_4);
            if (ad_value_low < temp_ad_Fire && ad_value_high > temp_ad_Fire)
            {
                if (ad_value_low < temp_ad_ch && ad_value_high > temp_ad_ch)
                {
                    if (abs(temp_ad_ch - temp_ad_Fire) < 40)
                    {
                        GPIO_BOP(GPIOA) = GPIO_PIN_11; //通路3开启指示灯亮 输出低电平
                        //open road 3
                        //output "01"
                        GPIO_BC(GPIOB) = GPIO_PIN_4;
                        GPIO_BOP(GPIOB) = GPIO_PIN_5;
                        road_ad_value(temp_ad_Fire, temp_ad_ch);
                        break;
                    }
                }
            }
            // road_ad_value(temp_ad_Fire, temp_ad_ch);
            // led_3_flash(1, 10);
            count++;
            if (count == 500)
            {
                count = 0;
                break;
            }
        }
    }
}
//关闭通路 channel
void Close_road(int channel)
{
    if (channel == 1)
    {
        GPIO_BC(GPIOB) = GPIO_PIN_3; //通路1关闭指示灯熄灭 输出低电平
        //CLOSED ROAD1 10
        GPIO_BOP(GPIOB) = GPIO_PIN_8;
        GPIO_BC(GPIOB) = GPIO_PIN_9;
    }
    if (channel == 2)
    {
        GPIO_BC(GPIOA) = GPIO_PIN_12; //通路2关闭指示灯熄灭 输出低电平
        //CLOSED ROAD2 10
        GPIO_BOP(GPIOB) = GPIO_PIN_6;
        GPIO_BC(GPIOB) = GPIO_PIN_7;
    }
    if (channel == 3)
    {
        GPIO_BC(GPIOA) = GPIO_PIN_11; //通路3关闭指示灯熄灭 输出低电平
        //CLOSED ROAD3 10
        GPIO_BOP(GPIOB) = GPIO_PIN_4;
        GPIO_BC(GPIOB) = GPIO_PIN_5;
    }
}
//继电器输出引脚复位 10
void Road_dc_reset()
{

    //turn off current //10
    for (i = 1; i < 4; i++)
    {
        Close_road(i);
    }

    delay_1ms(1000);

    for (i = 1; i < 4; i++)
    {
        Road_dc_close(i);
    }
}

//继电器 断开供电 00
void Road_dc_close(int channel)
{
    if (channel == 1)
    {
        GPIO_BC(GPIOB) = GPIO_PIN_8;
        GPIO_BC(GPIOB) = GPIO_PIN_9;
    }

    if (channel == 2)
    {
        GPIO_BC(GPIOB) = GPIO_PIN_6;
        GPIO_BC(GPIOB) = GPIO_PIN_7;
    }

    if (channel == 3)
    {
        GPIO_BC(GPIOB) = GPIO_PIN_4;
        GPIO_BC(GPIOB) = GPIO_PIN_5;
    }
}

//读取传感器数据
void Get_Im1281B_data()
{
    int i = 0;
    union crcdata
    {
        unsigned int word16;
        unsigned char byte[2];
    } crcnow;

    //发送取数据指令
    Tx_Buffer[0] = Read_ID; //模块的 ID 号，默认 ID 为 0x01
    Tx_Buffer[1] = 0x03;    //功能指令
    Tx_Buffer[2] = 0x00;    //指令操作开始寄存器地址高位
    Tx_Buffer[3] = 0x48;    //指令操作开始寄存器地址低位
    Tx_Buffer[4] = 0x00;    //要操作的寄存器数量，按起始寄存器地址算起，寄存器数量高位
    Tx_Buffer[5] = 0x08;    //寄存器数量低位
    crcnow.word16 = chkcrc(Tx_Buffer, 6);
    Tx_Buffer[6] = crcnow.byte[1]; //CRC 效验低字节在前
    Tx_Buffer[7] = crcnow.byte[0];
    for (i = 0; i < 8; i++)
    {
        usart_data_transmit(EVAL_COM1, Tx_Buffer[i]);
        while (RESET == usart_flag_get(EVAL_COM1, USART_FLAG_TBE))
            ;
    }

    //获取返回的数据
    delay_1ms(1);
    i = 0;
    rece_data_flag = SET;
    rece_data_count = 0;
    while (i < 37)
    {

        if (RESET != usart_flag_get(EVAL_COM1, USART_FLAG_RBNE))
        {
            RX_Buffer[i++] = usart_data_receive(EVAL_COM1);
        }
        if (rece_data_count == 2)
        {
            break;
        }
    }
    rece_data_flag = RESET;
}
/*
   计算生成CRC码
*/
unsigned int calccrc(unsigned char crcbuf, unsigned int crc)
{
    unsigned char i;
    unsigned char chk;
    crc = crc ^ crcbuf;
    for (i = 0; i < 8; i++)
    {
        chk = (unsigned char)(crc & 1);
        crc = crc >> 1;
        crc = crc & 0x7fff;
        if (chk == 1)
            crc = crc ^ 0xa001;
        crc = crc & 0xffff;
    }
    return crc;
}
/*
   检测校验码是否正确
*/
unsigned int chkcrc(unsigned char *buf, unsigned char len)
{
    unsigned char hi, lo;
    unsigned int i;
    unsigned int crc;
    crc = 0xFFFF;
    for (i = 0; i < len; i++)
    {
        crc = calccrc(*buf, crc);
        buf++;
    }
    hi = (unsigned char)(crc % 256);
    lo = (unsigned char)(crc / 256);
    crc = (((unsigned int)(hi)) << 8) | lo;
    return crc;
}

//校验读取的数据是否正确
int Check_Im1281B_data(void)
{
    unsigned char i;
    unsigned int temp_rx[8];
    union crcdata
    {
        unsigned int word16;
        unsigned char byte[2];
    } crcnow;
    int reveive_number = 37;
    if (RX_Buffer[0] == Read_ID)
    {                                                          //确认 ID 正确
        crcnow.word16 = chkcrc(RX_Buffer, reveive_number - 2); //reveive_number 是接收数据总长度
        if ((crcnow.byte[0] == RX_Buffer[reveive_number - 1]) && (crcnow.byte[1] == RX_Buffer[reveive_number - 2]))
        //确认 CRC 校验正确
        {
            // Serial.println("------- CRC校验通过--------");

            for (i = 0; i < 8; i++)
            {
                temp_rx[i] = (((unsigned long)(RX_Buffer[3 + i * 4])) << 24) | (((unsigned long)(RX_Buffer[4 + i * 4])) << 16) | (((unsigned long)(RX_Buffer[5 + i * 4])) << 8) | RX_Buffer[6 + i * 4];
            }
            for (i = 0; i < 4; i++)
            {
                if (temp_rx[i] > 0)
                    continue;
                else
                    return RESET;
            }

            Voltage_data = temp_rx[0] * 0.0001;                             //电压 单位：0.0001V
            Current_data = temp_rx[1] * 0.0001;                             //电流  单位 ：0.0001A
            Power_data = temp_rx[2] * 0.0001;                               // 有功功率  单位： 0.0001W
            Energy_data = temp_rx[3] * 0.0001;                              // 有功总能量  单位：0.0001KWh
            Pf_data = temp_rx[4] * 0.001;                                   // 功率因数  单位：0.001
            CO2_data = temp_rx[5] * 0.0001;                                 // 二氧化碳排量   单位：0.0001Kg
            Temperature_data = temp_rx[6] * 0.01;                           // 温度   单位：0.01C
            Fre_data = temp_rx[7] * 0.01;                                   //频率   单位：0.01Hz
            Reactive_data = Power_data * tan(acos(Pf_data));                //无功功率，计算得出
            if ((Temperature_data > 0) && (Fre_data > 48 && Fre_data < 51)) //数据再次确认，闪灯
            {
                //数据正确
                // led_flash(1);
                return SET;
            }
            else
            {
                return RESET; //数据不正确
            }
        }
        else
        {
            return RESET; //校验码错误
        }
    }
    else
    {
        return RESET; //设备ID错误
    }
}
int SaveData(void)
{
    if ((Voltage_data < Danger_Voltage) && (Current_data < Danger_Current))
    {
        // led_2_flash(1, 250);
        // printf("the single PF is %.3f\n", Pf_data);
        if (Pf_data >= 0.1)
        {
            led_2_flash(1, 250); //数据符合要求，保存

            Pf_data_buffer[SysNums_Getdata] = Pf_data;
            // Reactive_data_Buffer[SysNums_Getdata] = Reactive_data;
            Power_data_Buffer[SysNums_Getdata] = Power_data;
            return SET;
        }
        else
        { //过补偿保护，立刻切断所有电容通路
            if (Pf_data < 0)
            {
                for (i = 1; i < 4; i++)
                {
                    Close_road(i);
                }
                SysNums_Getdata = 0;
            }
            return RESET;
        }
    }
    else
    {
        //电路保护，立刻切断所有电容通路
        for (i = 1; i < 4; i++)
        {
            Close_road(i);
        }
        SysNums_Getdata = 0;
        return RESET;
    }
}
//计算电容方案无功容量
void Calculation_Capacity(void)
{
    double temp_Capac = 0;
    for (j = 0; j < 8; j++)
    {
        temp_Capac = 0;
        for (i = 0; i < 3; i++)
        {
            temp_Capac += Capacitive_reactive_1uF * Cpa_set_arr[j][i] * Capacitive_reactive_C[i];
        }
        Calculation_Capacity_plan[j] = temp_Capac;
    }
}

/*
   计算电容投切方案
*/
void Cal_Cpa_plan()
{
    double Average_Pf = 0, Average_Power = 0;
    double Total_Q = 0;
    double Ang1, Ang2;
    int i, j, k;
    Average_Pf = 0;
    Average_Power = 0;
    for (i = 0; i < Nums_Getdata; i++)
    {
        Average_Pf = Average_Pf + Pf_data_buffer[i];
        Average_Power = Average_Power + Power_data_Buffer[i];
    }
    Average_Pf = Average_Pf / Nums_Getdata;
    Average_Power = (Average_Power / Nums_Getdata); //单位：w

    // Average_Power = 1000;
    // Average_Pf = 0.5;
    // printf("\nAverage_PF is %.3f \n", Average_Pf);
    if (Average_Pf <= Expect_Pf && Average_Pf > 0)
    {
        Ang1 = acos(Average_Pf);
        Ang2 = acos(Expect_Pf);
        Current_Q = Average_Power * (tan(Ang1) - tan(Ang2));
        Total_Q = (Previous_Q + Current_Q);

        // printf("Current_Q is %.3f\n", Current_Q);
        // printf("Previous_Q is %.3f\n", Previous_Q);

        // Total_Q = Total_Q;
        // Total_Q = 1500;

        // printf("Total_Q is %.3f\n", Total_Q);
        for (i = 7; i >= 0; i--)
        {

            if (Total_Q >= Calculation_Capacity_plan[i])
            {
                Previous_Q = Calculation_Capacity_plan[i];
                // printf("the SET PLAN is  %d\n", i);
                for (j = 0; j < 3; j++)
                {
                    k = j + 1;
                    if (Cpa_set_arr[i][j] == 1)
                    {
                        Open_road(k);
                        // printf("Open road %d\n", j + 1);
                    }
                    else
                    {
                        Close_road(k);
                        // printf("Close road %d\n", j + 1);
                    }
                }
                delay_1ms(1000);
                for (k = 1; k < 4; k++)
                {
                    Road_dc_close(k);
                    // printf("Road_dc_close %d\n", k);
                }

                break;
            }
        }
    }
}

/*
    将AD采样结果发送到串口，只能在open road函数中使用
    fire：火线ad返回值
    cal：电容ad返回值
*/
void road_ad_value(int fire, int cal)
{
    // double real_fire, real_calc;
    // real_fire = (fire * 3.3) / 4096;
    // real_calc = (cal * 3.3) / 4096;
    // printf("the two ad results are :\n");
    // printf("Fire line: AD_RES:%d REAL_VOL %.3f\n", fire, real_fire);
    // printf("CalC line: AD_RES:%d REAL_VOL %.3f\n", cal, real_calc);
    // printf("Fire line:  REAL_VOL %.3f\n", real_fire);
    // printf("CalC line:  REAL_VOL %.3f\n", real_calc);
}
