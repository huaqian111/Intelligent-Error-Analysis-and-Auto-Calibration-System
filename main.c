/**
 ****************************************************************************************************
 * @file        main.c
 * @author      STM32H7 温度监控系统
 * @version     V6.0
 * @date        2026-07-09
 * @brief       主函数：系统初始化、按键调度、联网通信、风扇自动控制
 ****************************************************************************************************
 */

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"

/* 板级外设 */
#include "./BSP/SDRAM/sdram.h"
#include "./BSP/LED/led.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/KEY/key.h"
#include "./BSP/MPU/mpu.h"
#include "./BSP/TOUCH/touch.h"
#include "./BSP/DS18B20/ds18b20.h"
#include "./BSP/SPI/hardspi.h"
#include "./BSP/MAX31865/max31865_hardspi.h"
#include "./BSP/AP3216C/ap3216c.h"
#include "./BSP/ADC/adc.h"
#include "./BSP/DHT11/dht11.h"
#include "./BSP/F04A/F04A.h"
#include "./BSP/ESP8266/esp8266.h"

/* 应用层 */
#include "./app_ui.h"
#include "./app_algo.h"
#include "./global.h"

#include <stdio.h>
#include <string.h>

/* 全局变量与常量 */
SensorData_t g_sensor_data = {0};
DiagResult_t g_diag_result = {0};

extern uint8_t g_ui_mode;
extern uint8_t g_buzzer_mute;
extern uint8_t g_fault_sim;
static uint8_t previous_ui_mode = 0;

/* DHT11 读取计数（由主循环控制间隔） */
static uint8_t dht11_read_cnt = 0;

/* DS18B20 读取计数（降低读取频率） */
static uint8_t sensor_read_cnt = 0;

/* 发送定时器 */
static uint32_t last_led_tick = 0;
static uint32_t last_update_tick = 0;   // 上次完整更新时间
static uint32_t last_send_tick = 0;

/* 函数声明 */
static void Fan_Auto_Control(void);
static void Send_Sensor_Data_To_Server(void);
static void Check_And_Handle_Server_Command(void);

/*==============================================================================
 * 自动温控逻辑
 *============================================================================*/
static void Fan_Auto_Control(void)
{
    float sum_temp = 0;
    int count = 0;

    /* 收集有效温度（排除 -999.0f 等无效值） */
    if (g_sensor_data.ds18b20_temp > -50.0f) {
        sum_temp += g_sensor_data.ds18b20_temp;
        count++;
    }
    if (g_sensor_data.pt100_temp > -50.0f) {
        sum_temp += g_sensor_data.pt100_temp;
        count++;
    }
    if (g_sensor_data.dht11_temp > -50.0f) {
        sum_temp += g_sensor_data.dht11_temp;
        count++;
    }

    /* 没有有效温度传感器时关闭风扇 */
    if (count == 0) {
        g_sensor_data.fan.auto_active = 0;
        F04A_SetSpeed(0);
        return;
    }

    float avg_temp = sum_temp / count;

    /* 自动控制逻辑 */
    if (avg_temp > TEMP_HIGH) {
        g_sensor_data.fan.auto_active = 1;
    } else if (avg_temp < TEMP_LOW) {
        g_sensor_data.fan.auto_active = 0;
    }

    /* 执行风扇输出 */
    uint8_t fan_on = (g_sensor_data.fan.manual_enable || g_sensor_data.fan.auto_active);
    if (fan_on) {
        F04A_SetSpeed(g_sensor_data.fan.speed / 2);   /* PWM 减半 */
    } else {
        F04A_SetSpeed(0);
    }
}

/*==============================================================================
 * 远程命令处理
 *============================================================================*/
static void Check_And_Handle_Server_Command(void)
{
    if (strEsp8266_Fram_Record.InfBit.FramFinishFlag) {
        strEsp8266_Fram_Record.Data_RX_BUF[strEsp8266_Fram_Record.InfBit.FramLength] = '\0';
        char *cmd = strEsp8266_Fram_Record.Data_RX_BUF;

        if (strstr(cmd, "CMD:PAGE_MAIN"))          g_ui_mode = 0;
        else if (strstr(cmd, "CMD:PAGE_TREND"))    g_ui_mode = 1;
        else if (strstr(cmd, "CMD:PAGE_DETAIL"))   g_ui_mode = 2;
        else if (strstr(cmd, "CMD:PAGE_FAN"))      g_ui_mode = 3;
        else if (strstr(cmd, "CMD:TOGGLE_MUTE"))   g_buzzer_mute = !g_buzzer_mute;
        else if (strstr(cmd, "CMD:TOGGLE_FAULT"))  g_fault_sim = !g_fault_sim;
        else if (strstr(cmd, "CMD:RESET_HISTORY")) {
            for (int i = 0; i < 60; i++) C_APP_UI_UpdateHistory(25.0f, 25.0f);
            g_fault_sim = 0;
            g_buzzer_mute = 0;
        }
        else if (strstr(cmd, "CMD:FAN_SPEED_UP")) {
            if (g_sensor_data.fan.speed <= 90) g_sensor_data.fan.speed += 10;
            else g_sensor_data.fan.speed = 100;
        }
        else if (strstr(cmd, "CMD:FAN_SPEED_DOWN")) {
            if (g_sensor_data.fan.speed >= 10) g_sensor_data.fan.speed -= 10;
            else g_sensor_data.fan.speed = 0;
        }
        else if (strstr(cmd, "CMD:FAN_TOGGLE_MANUAL")) {
            g_sensor_data.fan.manual_enable = !g_sensor_data.fan.manual_enable;
        }

        strEsp8266_Fram_Record.InfBit.FramLength = 0;
        strEsp8266_Fram_Record.InfBit.FramFinishFlag = 0;
    }
}

/*==============================================================================
 * 发送数据到服务器
 *============================================================================*/
static void Send_Sensor_Data_To_Server(void)
{
    char buf[450];
    sprintf(buf,
        "DS18B20=%.1f,PT100=%.1f,INTERNAL=%.1f,VREF=%.2f,ALS=%d,PS=%d,IR=%d,"
        "DS18B20_ONLINE=%d,PT100_FAULT=%d,UI_MODE=%d,MUTE=%d,FAULT_SIM=%d,"
        "DHT11_TEMP=%.1f,DHT11_HUMI=%d,"
        "FAN_SPEED=%d,FAN_MANUAL=%d,FAN_AUTO=%d\r\n",
        g_sensor_data.ds18b20_temp,
        g_sensor_data.pt100_temp,
        g_sensor_data.internal_temp,
        g_sensor_data.vref_voltage,
        g_sensor_data.als,
        g_sensor_data.ps,
        g_sensor_data.ir,
        g_sensor_data.ds18b20_online,
        g_sensor_data.pt100_fault,
        g_ui_mode, g_buzzer_mute, g_fault_sim,
        g_sensor_data.dht11_temp, (int)g_sensor_data.dht11_humi,
        g_sensor_data.fan.speed,
        g_sensor_data.fan.manual_enable,
        g_sensor_data.fan.auto_active
    );
    macESP8266_Usart("%s", buf);
}

/*==============================================================================
 * 主函数
 *============================================================================*/
int main(void)
{
    char init_buf[200];

    /* 系统初始化 */
    sys_cache_enable();
    HAL_Init();
    sys_stm32_clock_init(160, 5, 2, 4);
    delay_init(400);
    usart_init(115200);
    mpu_memory_protection();
    led_init();
    sdram_init();
    lcd_init();
    key_init();
    tp_dev.init();

    /* 传感器初始化 */
    if (DS18B20_Init()) {
        lcd_clear(WHITE);
        lcd_show_string(30, 170, 200, 16, 16, "DS18B20 NOT FOUND!", RED);
        while (1);
    }
    MAX31865_Init_HardSPI();
    ap3216c_init();
    dht11_init();

    /* 风扇初始化 */
    F04A_Init();
    F04A_SetDirection(FAN_DIR_FORWARD);
    g_sensor_data.fan.speed = 50;
    g_sensor_data.fan.manual_enable = 0;
    g_sensor_data.fan.auto_active = 0;

    /* UI 初始化 */
    C_APP_UI_Init();

    /* 预采集数据 */
    Get_Data(&g_sensor_data);
    App_Read_AP3216C(&g_sensor_data);
    App_Read_DHT11(&g_sensor_data);
    B_APP_Diagnosis_Process(&g_sensor_data, &g_diag_result);
    C_APP_UI_UpdateHistory(g_sensor_data.ds18b20_temp, g_sensor_data.pt100_temp);

    /* ESP8266 联网 */
    ESP8266_Init();
    ESP8266_AT_Test();
    ESP8266_Net_Mode_Choose(STA);
    while (!ESP8266_JoinAP(macUser_ESP8266_ApSsid, macUser_ESP8266_ApPwd)) {
        HAL_Delay(1000);
    }
    ESP8266_Cmd("AT+CIFSR", "OK", 0, 1000);
    ESP8266_Cmd("AT+CIPMUX=0", "OK", 0, 500);
    ESP8266_Link_Server(enumTCP, "118.190.211.149", "8081", Single_ID_0);
    sprintf(init_buf, "SYSTEM: Temperature monitor online with fan control\r\n");
    ESP8266_SendString(DISABLE, init_buf, strlen(init_buf), Single_ID_0);
		/* 进入透传模式 */
		ESP8266_Cmd("AT+CIPMODE=1", "OK", 0, 500);      // 设置透传模式
		ESP8266_Cmd("AT+CIPSEND", ">", 0, 1000);         // 开始透传发送（等待 ">" 提示符）

    last_send_tick = HAL_GetTick();

    /* ======================== 主循环 ======================== */
    while (1)
    {
        /* ---- 触摸扫描 ---- */
        tp_dev.scan(0);
        if (tp_dev.sta & TP_PRES_DOWN) {
            C_APP_UI_SetTouch(tp_dev.x[0], tp_dev.y[0], 1);
        } else {
            C_APP_UI_SetTouch(0xFFFF, 0xFFFF, 0);
        }

        /* ---- 按键扫描 ---- */
        uint8_t key = key_scan(0);

				if (g_ui_mode == 3) {
						/* ===== 风扇控制页面 ===== */
						if (key == KEY0_PRES) {
								/* 加速 */
								if (g_sensor_data.fan.speed <= 90) g_sensor_data.fan.speed += 10;
								else g_sensor_data.fan.speed = 100;
								C_APP_UI_ShowFanControl(&g_sensor_data, &g_diag_result);
							  last_update_tick = HAL_GetTick();
						} else if (key == KEY1_PRES) {
								/* 手动开关 */
								g_sensor_data.fan.manual_enable = !g_sensor_data.fan.manual_enable;
								C_APP_UI_ShowFanControl(&g_sensor_data, &g_diag_result);
							  last_update_tick = HAL_GetTick();
						} else if (key == KEY2_PRES) {
								/* 减速 */
								if (g_sensor_data.fan.speed >= 10) g_sensor_data.fan.speed -= 10;
								else g_sensor_data.fan.speed = 0;
								C_APP_UI_ShowFanControl(&g_sensor_data, &g_diag_result);
							  last_update_tick = HAL_GetTick();
						} else if (key == WKUP_PRES) {
								/* 返回之前的页面 */
								g_ui_mode = previous_ui_mode;
						}
				} else {
						/* ===== 其他页面 ===== */
						switch (g_ui_mode) {
								case 0: /* 主页面 */
										if (key == KEY0_PRES) {
												g_ui_mode = 1;                         /* 下一页到趋势图 */
										} else if (key == KEY1_PRES) {
												g_buzzer_mute = !g_buzzer_mute;
										} else if (key == KEY2_PRES) {
												g_fault_sim = !g_fault_sim;                       /* 关闭故障模拟 */
										} else if (key == WKUP_PRES) {
												previous_ui_mode = 0;
												g_ui_mode = 3;
										}
										break;
								case 1: /* 趋势图 */
										if (key == KEY0_PRES) {
												g_ui_mode = 2;                         /* 下一页到详细信息 */
										} else if (key == KEY1_PRES) {
												C_APP_UI_ResetHistory();
										} else if (key == KEY2_PRES) {
												g_ui_mode = 0;                         /* 上一页到主页面 */
										} else if (key == WKUP_PRES) {
												previous_ui_mode = 1;
												g_ui_mode = 3;
										}
										break;
								case 2: /* 详细信息 */
										if (key == KEY0_PRES) {
												/* 无作用 */
										} else if (key == KEY1_PRES) {
												/* 无作用 */
										} else if (key == KEY2_PRES) {
												g_ui_mode = 1;                         /* 上一页到趋势图 */
										} else if (key == WKUP_PRES) {
												previous_ui_mode = 2;
												g_ui_mode = 3;
										}
										break;
								default:
										break;
						}
				}

				if (HAL_GetTick() - last_update_tick >= 200) {
						last_update_tick = HAL_GetTick();
						/* ---- 数据采集（通过 app_algo 接口） ---- */
						if (g_fault_sim) {
								App_Generate_Simulated_Data(&g_sensor_data);
						} else {
								/* DS18B20 / PT100 / MCU 内部间隔读取 */
								sensor_read_cnt++;
								if (sensor_read_cnt >= DS18B20_READ_INTERVAL) {
										sensor_read_cnt = 0;
										Get_Data(&g_sensor_data);
								}
								/* AP3216C 每次循环都读取 */
								App_Read_AP3216C(&g_sensor_data);
								/* DHT11 间隔读取 */
								dht11_read_cnt++;
								if (dht11_read_cnt >= DHT11_READ_INTERVAL) {
										dht11_read_cnt = 0;
										App_Read_DHT11(&g_sensor_data);
								}
						}

						/* ---- 自动温控 ---- */
						Fan_Auto_Control();

						/* ---- 诊断 ---- */
						B_APP_Diagnosis_Process(&g_sensor_data, &g_diag_result);

						/* ---- 更新温度历史 ---- */
						C_APP_UI_UpdateHistory(g_sensor_data.ds18b20_temp, g_sensor_data.pt100_temp);

						/* ---- 远程命令处理 ---- */
						Check_And_Handle_Server_Command();

						/* ---- 刷新 LCD UI ---- */
						switch (g_ui_mode) {
								case 0: C_APP_UI_ShowMain(&g_sensor_data, &g_diag_result); break;
								case 1: C_APP_UI_ShowTrend(&g_sensor_data);                break;
								case 2: C_APP_UI_ShowDetail(&g_sensor_data, &g_diag_result); break;
								case 3: C_APP_UI_ShowFanControl(&g_sensor_data, &g_diag_result); break;
								default: break;
						}
				}
				/* ---- 每 1000ms 发送一次数据到服务器（与 500ms 错开） ---- */
				if (HAL_GetTick() - last_send_tick >= 1000) {
						last_send_tick = HAL_GetTick();
						Send_Sensor_Data_To_Server();
				}
				/* ---- LED 指示 ---- */
				if (HAL_GetTick() - last_led_tick >= 1000) {
						LED0_TOGGLE();
						last_led_tick = HAL_GetTick();
				}
		}
}