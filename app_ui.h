/**
 * @file        app_ui.h
 * @brief       用户界面模块 - 头文件
 * @details     声明所有 UI 显示函数、触摸设置函数和历史数据更新函数
 * @version     V5.0
 * @date        2026-07-09
 * @note        与 app_ui.c 配套使用
 */
#ifndef __APP_UI_H
#define __APP_UI_H

#include "./SYSTEM/sys/sys.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/LED/led.h"
#include "global.h"

extern uint8_t g_ui_mode;          /* 0-主界面，1-趋势图，2-详细信息，3-风扇控制 */
extern uint8_t g_buzzer_mute;      /* 1=静音 */
extern uint8_t g_fault_sim;        /* 1=故障模拟 */

void C_APP_UI_Init(void);
void C_APP_UI_ShowMain(SensorData_t *data, DiagResult_t *result);
void C_APP_UI_ShowTrend(SensorData_t *data);
void C_APP_UI_ShowDetail(SensorData_t *data, DiagResult_t *result);
void C_APP_UI_ShowFanControl(SensorData_t *data, DiagResult_t *result);
void C_APP_UI_SetTouch(uint16_t x, uint16_t y, uint8_t pressed);
void C_APP_UI_UpdateHistory(float ds18b20_temp, float pt100_temp);
void C_APP_UI_ResetHistory(void);

#endif /* __APP_UI_H */