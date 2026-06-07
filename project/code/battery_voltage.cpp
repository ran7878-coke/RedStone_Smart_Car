#include "zf_common_headfile.hpp"

uint16  adc_reg[20]  = {0};      // ADC采集原始数字量
uint16  adc_reg_ar   = 0;        //ADC采集平均值
float   adc_scale    = 0.0f;     // ADC电压校准系数 (单位: mv/bit)
uint16  battery_vol  = 0;        // 电池实际电压值 (单位: mv 毫伏)
uint8 i = 0;

// 初始化ADC对象，指定电池分压检测通道 ADC_CH7
zf_driver_adc battery_adc(ADC_CH7_PATH);

// 宏定义硬件参数 - 抽离为宏，方便修改，可读性强
#define BATTERY_DIV_RATIO      (11)    // 硬件分压比 (R37+R38)/R38 = 11

void adc_battery_Init(void)//电池电压读取初始化
{
    // ADC校准系数只读取一次，放在while循环外面，程序上电初始化时读1次即可
    adc_scale = battery_adc.get_scale();
}

uint16 battery_voltage_get(void) 
{
        // 采集ADC原始值
        if (i == 20)
            i = 0;
        adc_reg[i] = battery_adc.convert();
        i++;
        int sum = 0;
        uint8 num = 0;
        for(uint8 j = 0; j < 20; j++)
        {
            if (adc_reg[j] == 0)
                break;
            sum += adc_reg[j];
            num++;
        }
        adc_reg_ar = sum / num;
        
        // 电压计算公式(adc_reg*adc_scale=采样点电压mv  ×11=电池实际电压mv)
        battery_vol = adc_reg_ar * adc_scale * BATTERY_DIV_RATIO;
        
        //printf("adc_reg_ar = %d\r\n", adc_reg_ar);
        //printf("adc_scale = %f\r\n", adc_scale);
        //printf("battery vol = %d\r\n", battery_vol);

    return battery_vol;
}