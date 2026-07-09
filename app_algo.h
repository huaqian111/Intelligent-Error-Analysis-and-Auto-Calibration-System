#ifndef __APP_ALGO_H
#define __APP_ALGO_H

#include "global.h"

/**
 * @brief  读取 DS18B20、PT100、MCU内部温度和参考电压
 * @param  data : 指向传感器数据结构的指针
 */
void Get_Data(SensorData_t *data);

/**
 * @brief  读取 MCU 内部参考电压和芯片温度
 * @param  out_vdda : 输出实际电源电压 (V)
 * @param  out_temp : 输出芯片内部温度 (℃)
 */
void MCU_Get_Voltage_And_Temp(float *out_vdda, float *out_temp);

/**
 * @brief  读取 AP3216C 光环境传感器（ALS, PS, IR）
 * @param  data : 指向传感器数据结构的指针
 */
void App_Read_AP3216C(SensorData_t *data);

/**
 * @brief  读取 DHT11 温湿度传感器
 * @param  data : 指向传感器数据结构的指针
 * @retval 0 成功，其他失败
 */
uint8_t App_Read_DHT11(SensorData_t *data);

/**
 * @brief  生成模拟传感器数据（用于故障模拟模式）
 * @param  data : 指向传感器数据结构的指针
 */
void App_Generate_Simulated_Data(SensorData_t *data);

/**
 * @brief  诊断处理函数：计算健康分数并给出建议
 * @param  data   : 当前传感器数据指针
 * @param  result : 诊断结果输出指针
 */
void B_APP_Diagnosis_Process(SensorData_t *data, DiagResult_t *result);

#endif /* __APP_ALGO_H */