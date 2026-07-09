/**
 * @file        app_algo.c
 * @brief       传感器数值获取与诊断处理模块
 * @details     封装所有传感器读取函数（DS18B20、PT100、MCU内部、AP3216C、DHT11）
 *              以及模拟数据生成和诊断逻辑
 */
#include "app_algo.h"
#include "./SYSTEM/usart/usart.h"   /* 用于 printf 调试 */
#include "stm32h7xx_hal.h"
#include "./SYSTEM/sys/sys.h"
#include <string.h>
#include <math.h>

/* 传感器驱动头文件 */
#include "./BSP/DS18B20/ds18b20.h"
#include "./BSP/SPI/hardspi.h"
#include "./BSP/MAX31865/max31865_hardspi.h"
#include "./BSP/AP3216C/ap3216c.h"
#include "./BSP/DHT11/dht11.h"
#include "./BSP/ADC/adc.h"

/* STM32H743 校准数据寄存器地址 (16位) */
#define VREFINT_CAL_ADDR_ ((uint16_t *)((uint32_t)0x1FF1E860))
#define TS_CAL1_ADDR      ((uint16_t *)((uint32_t)0x1FF1E820))
#define TS_CAL2_ADDR      ((uint16_t *)((uint32_t)0x1FF1E840))

/* 第一次运行标志（用于诊断滤波） */
static uint8_t is_first_run = 1;
static float last_pt100_temp = 0.0f;

/*==============================================================================
 * 内部函数：读取 MCU 内部 ADC 原始值（专用于内部通道）
 *============================================================================*/
static uint32_t Get_MCU_Internal_ADC_Raw(uint32_t channel)
{
    ADC_HandleTypeDef hadc3 = {0};
    ADC_ChannelConfTypeDef sConfig = {0};
    uint32_t adc_val = 0;

    /* 配置 ADC 时钟源 */
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_CLKP;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
    __HAL_RCC_ADC3_CLK_ENABLE();

    /* 初始化 ADC3 */
    hadc3.Instance = ADC3;
    hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
    hadc3.Init.Resolution = ADC_RESOLUTION_16B;
    hadc3.Init.ScanConvMode = DISABLE;
    hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc3.Init.ContinuousConvMode = DISABLE;
    hadc3.Init.NbrOfConversion = 1;
    hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
    hadc3.Init.LowPowerAutoWait = DISABLE;
    if (HAL_ADC_Init(&hadc3) != HAL_OK) return 0;

    /* 退出深度掉电，启动调节器 */
    hadc3.Instance->CR &= ~ADC_CR_DEEPPWD;
    hadc3.Instance->CR |= ADC_CR_ADVREGEN;
    uint32_t wait_loop = (SystemCoreClock / 100000UL);
    while (wait_loop--) { __NOP(); }

    /* 自动校准 */
    if (HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
        __HAL_RCC_ADC3_CLK_DISABLE();
        return 0;
    }

    /* 配置通道 */
    sConfig.Channel = (channel == ADC_CHANNEL_TEMPSENSOR) ? ADC_CHANNEL_TEMPSENSOR : ADC_CHANNEL_VREFINT;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
        __HAL_RCC_ADC3_CLK_DISABLE();
        return 0;
    }

    /* 使能内部通道 */
    if (channel == ADC_CHANNEL_TEMPSENSOR)      ADC3_COMMON->CCR |= ADC_CCR_TSEN;
    else if (channel == ADC_CHANNEL_VREFINT)    ADC3_COMMON->CCR |= ADC_CCR_VREFEN;

    /* 启动转换并读取 */
    HAL_ADC_Start(&hadc3);
    if (HAL_ADC_PollForConversion(&hadc3, 50) == HAL_OK) {
        adc_val = HAL_ADC_GetValue(&hadc3);
    }
    HAL_ADC_Stop(&hadc3);

    /* 关闭内部通道并释放外设 */
    ADC3_COMMON->CCR &= ~(ADC_CCR_TSEN | ADC_CCR_VREFEN);
    HAL_ADC_DeInit(&hadc3);
    __HAL_RCC_ADC3_CLK_DISABLE();
    return adc_val;
}

/*==============================================================================
 * 读取 MCU 内部电压和温度（公开接口）
 *============================================================================*/
void MCU_Get_Voltage_And_Temp(float *out_vdda, float *out_temp)
{
    uint32_t vref_raw = Get_MCU_Internal_ADC_Raw(ADC_CHANNEL_VREFINT);
    uint32_t temp_raw = Get_MCU_Internal_ADC_Raw(ADC_CHANNEL_TEMPSENSOR);

    uint16_t vref_cal = *VREFINT_CAL_ADDR_;
    uint16_t ts_cal1  = *TS_CAL1_ADDR;
    uint16_t ts_cal2  = *TS_CAL2_ADDR;

    /* 计算实际 VDDA */
    if (vref_raw != 0) {
        *out_vdda = 3.3f * (float)vref_cal / (float)vref_raw;
    } else {
        *out_vdda = 3.3f;
    }

    /* 计算芯片温度 */
    if (ts_cal2 != ts_cal1) {
        float temp_raw_compensated = (float)temp_raw * (*out_vdda / 3.3f);
        *out_temp = ((110.0f - 30.0f) / (float)(ts_cal2 - ts_cal1)) * (temp_raw_compensated - (float)ts_cal1) + 30.0f;
    } else {
        *out_temp = 0.0f;
    }
}

/*==============================================================================
 * 读取 DS18B20、PT100、MCU内部（原有 Get_Data）
 *============================================================================*/
void Get_Data(SensorData_t *data)
{
    uint8_t read_cfg = 0;

    /* PT100 */
    MAX31865_WriteRegister_HardSPI(MAX31865_CONFIG_REG, 0xD3);
    delay_ms(10);
    MAX31865_WriteRegister_HardSPI(MAX31865_CONFIG_REG, 0xD1);

    HSPI1_CS1(0);
    HSPI1_Read_Write_Byte(MAX31865_CONFIG_REG & 0x7F);
    read_cfg = HSPI1_Read_Write_Byte(0xFF);
    HSPI1_CS1(1);

    if (read_cfg == 0xD1) {
        delay_ms(20);
        data->pt100_temp   = MAX31865_Get_Temperature_HardSPI();
        data->pt100_fault  = MAX31865_Read_Fault_Status();
    } else {
        data->pt100_fault  = 0xFF;
        data->pt100_temp   = -999.0f;
    }

    /* DS18B20 */
    if (DS18B20_Init() == 0) {
        data->ds18b20_online = 1;
        short temp_raw = DS18B20_Get_Temp();
        data->ds18b20_temp   = (float)temp_raw / 10.0f;
    } else {
        data->ds18b20_online = 0;
        data->ds18b20_temp   = -999.0f;
    }

    /* MCU 内部 */
    MCU_Get_Voltage_And_Temp(&data->vref_voltage, &data->internal_temp);
}

/*==============================================================================
 * 读取 AP3216C 光环境传感器
 *============================================================================*/
void App_Read_AP3216C(SensorData_t *data)
{
    ap3216c_read_data(&data->ir, &data->ps, &data->als);
}

/*==============================================================================
 * 读取 DHT11 温湿度
 *============================================================================*/
uint8_t App_Read_DHT11(SensorData_t *data)
{
    uint8_t temp_u8, humi_u8;
    if (dht11_read_data(&temp_u8, &humi_u8) == 0) {
        data->dht11_temp = (float)temp_u8;
        data->dht11_humi = humi_u8;
        return 0;
    }
    return 1;
}

/*==============================================================================
 * 生成模拟传感器数据（故障模拟模式）
 *============================================================================*/
void App_Generate_Simulated_Data(SensorData_t *data)
{
    static float temp = 25.0f;
    static uint8_t direction = 1;

    temp += direction * 0.1f;
    if (temp > 35.0f) direction = 0;
    if (temp < 15.0f) direction = 1;

    data->ds18b20_temp   = temp;
    data->ds18b20_online = 1;
    data->pt100_temp     = temp + 0.3f;
    data->pt100_fault    = 0;
    data->vref_voltage   = 3.30f;
    data->internal_temp  = 32.0f + temp * 0.1f;

    /* 如果故障模拟标志被设置，产生异常值 */
    extern uint8_t g_fault_sim;   /* 在 app_ui.c 中定义 */
    if (g_fault_sim) {
        data->ds18b20_temp   = -5.0f;
        data->ds18b20_online = 0;
        data->pt100_temp     = -5.0f;
        data->pt100_fault    = 1;
    }
}

/*==============================================================================
 * 诊断处理（计算健康分数和建议）
 *============================================================================*/
void B_APP_Diagnosis_Process(SensorData_t *data, DiagResult_t *result)
{
    if (is_first_run) {
        if (!data->pt100_fault) last_pt100_temp = data->pt100_temp;
        is_first_run = 0;
    }

    float filtered_pt100 = data->pt100_temp;
    if (data->pt100_fault == 0) {
        float alpha = 0.90f;
        filtered_pt100 = alpha * data->pt100_temp + (1.0f - alpha) * last_pt100_temp;
    }

    int current_score = 100;
    char log_msg[64] = "System Normal";

    if (data->ds18b20_online == 0 && data->pt100_fault != 0) {
        current_score = 0;
        strcpy(log_msg, "CRITICAL: Both disconnected!");
    } else {
        if (data->ds18b20_online == 0 && data->pt100_fault == 0) {
            current_score -= 40;
            strcpy(log_msg, "WARN: DS18B20 wire broken!");
        }
        if (data->pt100_fault != 0 && data->ds18b20_online != 0) {
            current_score -= 40;
            strcpy(log_msg, "WARN: PT100 fault!");
        }
        if (data->ds18b20_online != 0 && data->pt100_fault == 0) {
            float temp_diff = fabs(filtered_pt100 - data->ds18b20_temp);
            if (temp_diff > 5.0f) {
                current_score -= 25;
                strcpy(log_msg, "HINT: Large temp diff");
            }
        }
        if (data->pt100_fault == 0) {
            float mutation = fabs(data->pt100_temp - last_pt100_temp);
            if (mutation > 15.0f) {
                current_score -= 15;
                strcpy(log_msg, "WARN: Mutation noise");
            }
        }
        if (data->vref_voltage < 3.13f || data->vref_voltage > 3.46f) {
            current_score -= 15;
            strcpy(log_msg, "WARN: Voltage abnormal!");
        }
    }

    if (current_score < 0)   current_score = 0;
    if (current_score > 100) current_score = 100;

    result->health_score = (uint8_t)current_score;
    strcpy(result->advice_str, log_msg);

    if (result->health_score >= 90)      result->error_code = 0;   // GREEN
    else if (result->health_score >= 60) result->error_code = 1;   // YELLOW
    else                                 result->error_code = 2;   // RED

    if (data->pt100_fault == 0) {
        last_pt100_temp = filtered_pt100;
    }

    /* 串口输出调试信息 */
    printf("\r\n%.2f,%.2f,%d\r\n",
           data->ds18b20_online ? data->ds18b20_temp : 0.0f,
           data->pt100_fault != 0 ? 0.0f : filtered_pt100,
           result->health_score);
}