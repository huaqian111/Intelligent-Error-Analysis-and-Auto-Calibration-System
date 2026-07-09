/**
 * @file        app_ui.c
 * @brief       用户界面模块 - 实现文件
 * @details     实现各 UI 页面的绘制、触摸处理、蜂鸣器控制及温度历史管理
 * @version     V1.0
 * @date        2026-07-09
 * @note        所有函数均为阻塞式绘图，主循环中按需调用
 */

#include "app_ui.h"                /* 本模块头文件 */
#include "./SYSTEM/delay/delay.h"
#include "./BSP/IIC/myiic.h"
#include <stdio.h>
#include <string.h>

/*==============================================================================
 * 全局变量定义
 *============================================================================*/

uint8_t g_ui_mode = 0;            /**< 当前 UI 模式，默认主界面 */
uint8_t g_buzzer_mute = 0;        /**< 蜂鸣器静音，默认正常 */
uint8_t g_fault_sim = 0;          /**< 故障模拟，默认关闭 */

/** @brief 温度历史缓冲区长度（60个点） */
#define TEMP_HISTORY_LEN 60

static float   temp_history[TEMP_HISTORY_LEN] = {0}; /**< 温度历史环形缓冲区 */
static uint8_t temp_history_idx = 0;                 /**< 当前写入索引 */
static uint8_t temp_history_count = 0;               /**< 已存储的有效点数 */

static float pt100_history[TEMP_HISTORY_LEN] = {0};
static uint8_t pt100_history_idx = 0;
static uint8_t pt100_history_count = 0;

/** @brief 触摸状态变量 */
static uint16_t touch_x = 0xFFFF;   /**< 触摸 X 坐标（无效值标记） */
static uint16_t touch_y = 0xFFFF;   /**< 触摸 Y 坐标 */
static uint8_t  touch_pressed = 0;  /**< 触摸按下标志 */

/*==============================================================================
 * PCF8574 IIC IO 扩展芯片驱动（用于蜂鸣器控制）
 *============================================================================*/

/** @brief PCF8574 设备地址（ADDR 接地时） */
#define PCF8574_ADDR        0x20

/** @brief 蜂鸣器控制位（P0 引脚） */
#define BEEP_PIN            0x01

/**
 * @brief  向 PCF8574 写入一个字节（控制 IO 输出）
 * @param  data : 要写入的数据字节
 */
static void PCF8574_WriteByte(uint8_t data)
{
    iic_start();
    iic_send_byte(PCF8574_ADDR << 1);   /* 写地址 */
    iic_wait_ack();
    iic_send_byte(data);                 /* 发送控制数据 */
    iic_wait_ack();
    iic_stop();
}

/**
 * @brief  从 PCF8574 读取当前 IO 状态
 * @return 读取到的字节（IO 电平）
 */
static uint8_t PCF8574_ReadByte(void)
{
    uint8_t data;
    iic_start();
    iic_send_byte((PCF8574_ADDR << 1) | 0x01);  /* 读地址 */
    iic_wait_ack();
    data = iic_read_byte(0);                     /* 读取（不回 ACK） */
    iic_stop();
    return data;
}

/**
 * @brief  控制蜂鸣器开关（受静音标志影响）
 * @param  on : 1-打开，0-关闭
 */
void C_BEEP_Set(uint8_t on)
{
    /* 若全局静音则强制关闭 */
    if (g_buzzer_mute) {
        on = 0;
    }

    uint8_t val = PCF8574_ReadByte();
    if (on) {
        val &= ~BEEP_PIN;        /* P0=0 => 蜂鸣器响（低电平驱动） */
    } else {
        val |= BEEP_PIN;         /* P0=1 => 蜂鸣器关 */
    }
    PCF8574_WriteByte(val);
}

/*==============================================================================
 * 触摸状态设置
 *============================================================================*/

/**
 * @brief  设置当前触摸点坐标和按下状态（由 main.c 触摸扫描调用）
 * @param  x       : 触摸 X 坐标
 * @param  y       : 触摸 Y 坐标
 * @param  pressed : 1-按下，0-松开
 */
void C_APP_UI_SetTouch(uint16_t x, uint16_t y, uint8_t pressed)
{
    touch_x = x;
    touch_y = y;
    touch_pressed = pressed;
}

/*==============================================================================
 * 内部函数：绘制健康状态条
 *============================================================================*/

/**
 * @brief  在 LCD 上绘制水平健康状态条
 * @param  x      : 左上角 X 坐标
 * @param  y      : 左上角 Y 坐标
 * @param  width  : 总宽度（像素）
 * @param  height : 总高度（像素）
 * @param  score  : 健康分数（0~100）
 */
static void Draw_Health_Bar(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t score)
{
    uint16_t color;

    /* 根据分数选择颜色 */
    if (score >= 90) {
        color = GREEN;          /* 优秀 */
    } else if (score >= 60) {
        color = YELLOW;         /* 警告 */
    } else {
        color = RED;            /* 故障 */
    }

    /* 绘制外框 */
    lcd_draw_rectangle(x, y, x + width, y + height, BLACK);

    /* 填充内部（留 1 像素边距） */
    uint16_t fill_w = (width - 2) * score / 100;
    if (fill_w > 0) {
        lcd_fill(x + 1, y + 1, x + fill_w, y + height - 1, color);
    }
}

/*==============================================================================
 * 内部函数：报警控制（LED + 蜂鸣器）
 *============================================================================*/

/**
 * @brief  根据健康分数控制 LED 和蜂鸣器报警
 * @param  score : 健康分数（0~100）
 */
static void C_Control_Alarm(uint8_t score)
{
    /* 非主界面时关闭所有报警 */
    if (g_ui_mode != 0) {
        LED0(1); LED1(1);   /* 熄灭 */
        C_BEEP_Set(0);      /* 蜂鸣器关闭 */
        return;
    }

    if (score >= 90) {
        /* 健康：绿灯亮，蜂鸣器关 */
        LED0(1);
        LED1(0);
        C_BEEP_Set(0);
    } else if (score >= 60) {
        /* 警告：绿灯亮，蜂鸣器交替响 */
        LED0(1);
        LED1(1);
        static uint8_t beep_toggle = 0;
        beep_toggle = !beep_toggle;
        C_BEEP_Set(beep_toggle);
    } else {
        /* 故障：红灯亮，绿灯交替闪，蜂鸣器持续响 */
        LED0(0);
        static uint8_t led_toggle = 0;
        led_toggle = !led_toggle;
        LED1(led_toggle);
        C_BEEP_Set(1);
    }
}

/*==============================================================================
 * 温度历史更新
 *============================================================================*/

/**
 * @brief  向温度历史环形缓冲区添加一个新的温度点
 * @param  temp : 当前温度（摄氏度）
 */

void C_APP_UI_UpdateHistory(float ds18b20_temp, float pt100_temp)
{
    /* DS18B20 ?? */
    temp_history[temp_history_idx++] = ds18b20_temp;
    if (temp_history_idx >= TEMP_HISTORY_LEN) temp_history_idx = 0;
    if (temp_history_count < TEMP_HISTORY_LEN) temp_history_count++;
    /* PT100 ?? */
    pt100_history[pt100_history_idx++] = pt100_temp;
    if (pt100_history_idx >= TEMP_HISTORY_LEN) pt100_history_idx = 0;
    if (pt100_history_count < TEMP_HISTORY_LEN) pt100_history_count++;
}

/*==============================================================================
 * UI 主页面：设备诊断
 *============================================================================*/

/**
 * @brief  绘制设备诊断主页面
 * @param  data   : 当前传感器数据
 * @param  result : 诊断结果
 */
void C_APP_UI_ShowMain(SensorData_t *data, DiagResult_t *result)
{
    char buf[30];

    lcd_clear(WHITE);
    lcd_draw_rectangle(5, 5, 234, 314, BLACK);
    lcd_show_string(10, 10, 200, 16, 16, "Device Diagnosis", RED);

    /* 温度显示 */
		lcd_show_string(10, 35, 100, 16, 16, "Temp:", BLUE);
		lcd_fill(80, 40, 230, 70, WHITE);
		float sum_temp = 0;
		int cnt = 0;
		if (data->ds18b20_temp > -50.0f) { sum_temp += data->ds18b20_temp; cnt++; }
		if (data->pt100_temp > -50.0f)   { sum_temp += data->pt100_temp;   cnt++; }
		if (data->dht11_temp > -50.0f)   { sum_temp += data->dht11_temp;   cnt++; }
		float avg_temp = (cnt > 0) ? (sum_temp / cnt) : -999.0f;
		sprintf(buf, "%.1f C", avg_temp);
		lcd_show_string(80, 35, 150, 32, 32, buf, BLUE);

    /* 状态文字 */
    char *status_str;
    uint16_t color;
    if (result->health_score >= 90) {
        status_str = "HEALTHY";
        color = GREEN;
    } else if (result->health_score >= 60) {
        status_str = "WARNING";
        color = YELLOW;
    } else {
        status_str = "FAULT";
        color = RED;
    }
    lcd_fill(90, 135, 230, 155, WHITE);
    lcd_show_string(90, 135, 200, 16, 16, status_str, color);
		
		/* 健康条 */
    lcd_show_string(10, 75, 100, 16, 16, "Health score:", BLUE);
    Draw_Health_Bar(20, 110, 200, 20, result->health_score);
		
		/* 分数 */
    sprintf(buf, "%d", result->health_score);
    lcd_show_string(115, 75, 200, 16, 16, buf, color);

    /* 建议 */
    lcd_show_string(10, 165, 100, 16, 16, "Advice:", BLACK);
    lcd_fill(20, 205, 230, 225, WHITE);
    lcd_show_string(20, 185, 210, 16, 16, result->advice_str, BLACK);

    /* 底部提示 */
    lcd_show_string(10, 270, 200, 16, 16, "KEY0:Next  KEY1:MUTE", BLACK);
		lcd_show_string(10, 290, 200, 16, 16, "KEY2:Fault KEY_UP:FS", BLACK);
    if (g_buzzer_mute) {
        lcd_show_string(137, 270, 50, 16, 16, "MUTE", RED);
    }

    /* 报警控制 */
    C_Control_Alarm(result->health_score);
}

/*==============================================================================
 * UI 页面2：温度趋势图
 *============================================================================*/

/**
 * @brief  绘制温度趋势折线图
 * @param  data : 当前传感器数据（用于显示最新值）
 */
void C_APP_UI_ShowTrend(SensorData_t *data)
{
    char buf[30];
    lcd_clear(WHITE);
    lcd_draw_rectangle(5, 5, 234, 314, BLACK);
    lcd_show_string(10, 10, 200, 16, 16, "Temperature Trend", BLUE);

    /* ?????? */
    float sum = 0.0f;
    int cnt = 0;
    if (data->ds18b20_temp > -50.0f) { sum += data->ds18b20_temp; cnt++; }
    if (data->pt100_temp   > -50.0f) { sum += data->pt100_temp;   cnt++; }
    if (data->dht11_temp   > -50.0f) { sum += data->dht11_temp;   cnt++; }
    float avg = (cnt > 0) ? (sum / cnt) : -999.0f;
    sprintf(buf, "AvgTemp: %.1f C", avg);
    lcd_show_string(105, 10, 120, 16, 16, buf, RED);

    /* ??? */
    lcd_draw_line(30, 260, 220, 260, BLACK);   /* X ? */
    lcd_draw_line(30, 30, 30, 260, BLACK);     /* Y ? */
    lcd_show_string(5, 30,  30, 16, 12, "40",   BLACK);
    lcd_show_string(5, 90,  30, 16, 12, "20",   BLACK);
    lcd_show_string(5, 150, 30, 16, 12, "0",    BLACK);
    lcd_show_string(5, 210, 30, 16, 12, "-20",  BLACK);
		lcd_show_string(5, 240, 30, 16, 12, "-30",  BLACK);
    lcd_fill(31, 31, 219, 259, WHITE);          /* ?????? */

/* ---------- ?? DS18B20 ??(??) ---------- */
    uint16_t x0 = 30, y0 = 260;
    uint8_t valid_start = 0;
    int count = (temp_history_count < TEMP_HISTORY_LEN) ? temp_history_count : TEMP_HISTORY_LEN;
    const float scale = 230.0f / 60.0f;   /* ???? -20~40 => 60? */

    for (int i = 0; i < count; i++) {
        float temp = temp_history[i];
        uint16_t x1 = 30 + (i * 190) / TEMP_HISTORY_LEN;
        int y_offset = (int)((temp + 20.0f) * scale);
        if (y_offset < 0)  y_offset = 0;
        if (y_offset > 230) y_offset = 230;
        uint16_t y1 = 260 - y_offset;
        if (valid_start) {
            lcd_draw_line(x0, y0, x1, y1, RED);
        }
        x0 = x1; y0 = y1; valid_start = 1;
    }

/* ---------- ?? PT100 ??(??) ---------- */
    x0 = 30; y0 = 260; valid_start = 0;
    int pt_count = (pt100_history_count < TEMP_HISTORY_LEN) ? pt100_history_count : TEMP_HISTORY_LEN;
    for (int i = 0; i < pt_count; i++) {
        float temp = pt100_history[i];
        uint16_t x1 = 30 + (i * 190) / TEMP_HISTORY_LEN;
        int y_offset = (int)((temp + 20.0f) * scale);
        if (y_offset < 0)  y_offset = 0;
        if (y_offset > 230) y_offset = 230;
        uint16_t y1 = 260 - y_offset;
        if (valid_start) {
            lcd_draw_line(x0, y0, x1, y1, BLUE);
        }
        x0 = x1; y0 = y1; valid_start = 1;
    }

    /* ???? */
    lcd_show_string(10, 270, 200, 16, 16, "KEY0:Next KEY1:Clc", BLACK);
		lcd_show_string(10, 290, 200, 16, 16, "KEY2:Last KEY_UP:FS", BLACK);
}

/*==============================================================================
 * UI 页面3：详细信息
 *============================================================================*/

/**
 * @brief  绘制传感器详细信息页面
 * @param  data   : 当前传感器数据
 * @param  result : 诊断结果
 */
void C_APP_UI_ShowDetail(SensorData_t *data, DiagResult_t *result)
{
    char buf[30];
    lcd_clear(WHITE);
    lcd_draw_rectangle(5, 5, 234, 314, BLACK);
    lcd_show_string(10, 10, 200, 16, 16, "Diagnosis Detail", RED);

    /* 左侧标签列 */
    uint16_t lx = 10, ly = 40;
    lcd_show_string(lx, ly, 100, 16, 16, "DS18B20:", BLACK);   ly += 20;
    lcd_show_string(lx, ly, 100, 16, 16, "PT100:", BLACK);     ly += 20;
    lcd_show_string(lx, ly, 100, 16, 16, "VREF:", BLACK);      ly += 20;
    lcd_show_string(lx, ly, 100, 16, 16, "MCU Temp:", BLACK);  ly += 20;
    lcd_show_string(lx, ly, 100, 16, 16, "ALS:", BLACK);       ly += 20;
    lcd_show_string(lx, ly, 100, 16, 16, "PS:", BLACK);        ly += 20;
    lcd_show_string(lx, ly, 100, 16, 16, "IR:", BLACK);        ly += 20;
    lcd_show_string(lx, ly, 100, 16, 16, "DHT11 Temp:", BLACK);ly += 20;
    lcd_show_string(lx, ly, 100, 16, 16, "DHT11 Humi:", BLACK);

    /* 右侧数值列 */
    uint16_t rx = 110, ry = 40;
    sprintf(buf, "%.1f C %s", data->ds18b20_temp, data->ds18b20_online ? "ON" : "OFF");
    lcd_show_string(rx, ry, 120, 16, 16, buf, data->ds18b20_online ? GREEN : RED); ry += 20;
    sprintf(buf, "%.1f C %s", data->pt100_temp, data->pt100_fault ? "OFF" : "ON");
    lcd_show_string(rx, ry, 120, 16, 16, buf, data->pt100_fault ? RED : GREEN); ry += 20;
    sprintf(buf, "%.2f V", data->vref_voltage);
    lcd_show_string(rx, ry, 120, 16, 16, buf, BLACK); ry += 20;
    sprintf(buf, "%.1f C", data->internal_temp);
    lcd_show_string(rx, ry, 120, 16, 16, buf, BLACK); ry += 20;
    sprintf(buf, "%d", data->als);
    lcd_show_string(rx, ry, 120, 16, 16, buf, BLACK); ry += 20;
    sprintf(buf, "%d", data->ps);
    lcd_show_string(rx, ry, 120, 16, 16, buf, BLACK); ry += 20;
    sprintf(buf, "%d", data->ir);
    lcd_show_string(rx, ry, 120, 16, 16, buf, BLACK); ry += 20;
    sprintf(buf, "%.1f C", data->dht11_temp);
    lcd_show_string(rx, ry, 120, 16, 16, buf, BLACK); ry += 20;
    sprintf(buf, "%d %%", data->dht11_humi);
    lcd_show_string(rx, ry, 120, 16, 16, buf, BLACK);

    /* 分隔线 */
    lcd_draw_line(10, 225, 225, 225, GRAY);

    /* 诊断分数和建议 */
    sprintf(buf, "Score: %d", result->health_score);
    lcd_show_string(10, 235, 200, 16, 16, buf,
                    result->health_score >= 90 ? GREEN : (result->health_score >= 60 ? YELLOW : RED));
    lcd_show_string(10, 265, 220, 16, 16, result->advice_str, BLACK);
    lcd_show_string(10, 290, 200, 16, 16, "KEY2:Last KEY_UP:FS", BLACK);
}

/*==============================================================================
 * UI 页面4：风扇控制（支持触摸操作）
 *============================================================================*/

/**
 * @brief  绘制风扇控制页面，包含手动/自动状态、转速条、开机/关机按钮
 * @param  data   : 当前传感器数据
 * @param  result : 诊断结果（用于未来扩展，此处未使用）
 */
void C_APP_UI_ShowFanControl(SensorData_t *data, DiagResult_t *result)
{
    char buf[30];
    static uint8_t last_touch_pressed = 0;  /**< 上次触摸按下标志，用于边缘检测 */

    lcd_clear(WHITE);
    lcd_draw_rectangle(5, 5, 234, 314, BLACK);
    lcd_show_string(10, 10, 200, 16, 16, "Fan Control", RED);

    /* 显示参考温度 */
    lcd_show_string(10, 40, 100, 16, 16, "Temp:", BLUE);
    sprintf(buf, "%.1f C", data->ds18b20_temp);
    lcd_show_string(60, 40, 150, 16, 16, buf, BLACK);

    /* 手动开关按钮 */
    uint16_t btn_x = 20, btn_y = 80, btn_w = 60, btn_h = 30;
    lcd_draw_rectangle(btn_x, btn_y, btn_x + btn_w, btn_y + btn_h, BLACK);
    if (data->fan.manual_enable) {
        lcd_fill(btn_x + 1, btn_y + 1, btn_x + btn_w - 1, btn_y + btn_h - 1, GREEN);
        lcd_show_string(btn_x + 10, btn_y + 8, 40, 16, 16, "ON", WHITE);
    } else {
        lcd_fill(btn_x + 1, btn_y + 1, btn_x + btn_w - 1, btn_y + btn_h - 1, GRAY);
        lcd_show_string(btn_x + 10, btn_y + 8, 40, 16, 16, "OFF", BLACK);
    }

    /* 转速进度条 */
    uint16_t bar_x = 20, bar_y = 140, bar_w = 180, bar_h = 20;
    lcd_draw_rectangle(bar_x, bar_y, bar_x + bar_w, bar_y + bar_h, BLACK);
    uint16_t fill_w = (uint16_t)((bar_w - 2) * data->fan.speed / 100);
    if (fill_w > 0) {
        lcd_fill(bar_x + 1, bar_y + 1, bar_x + fill_w, bar_y + bar_h - 1, BLUE);
    }
    sprintf(buf, "Speed: %d %%", data->fan.speed);
    lcd_show_string(20, 170, 180, 16, 16, buf, BLUE);

    /* 自动状态显示 */
    if (data->fan.auto_active) {
        lcd_show_string(20, 200, 200, 16, 16, "Auto: ON (Temp high)", RED);
    } else {
        lcd_show_string(20, 200, 200, 16, 16, "Auto: OFF", BLACK);
    }

    /* 底部提示 */
    lcd_show_string(10, 280, 200, 16, 16, "Touch: ON/OFF or drag bar", BLACK);

    /* ---- 触摸交互处理（边缘触发） ---- */
    if (touch_pressed) {
        /* 检测是否点击了开关按钮 */
        if (touch_x >= btn_x && touch_x <= btn_x + btn_w &&
            touch_y >= btn_y && touch_y <= btn_y + btn_h) {
            if (!last_touch_pressed) {
                data->fan.manual_enable = !data->fan.manual_enable;
                last_touch_pressed = 1;
            }
        }
        /* 检测是否在进度条上拖动 */
        else if (touch_x >= bar_x && touch_x <= bar_x + bar_w &&
                 touch_y >= bar_y && touch_y <= bar_y + bar_h) {
            if (!last_touch_pressed) {
                uint16_t pos = touch_x - bar_x;
                if (pos > bar_w) pos = bar_w;
                data->fan.speed = (uint8_t)(pos * 100 / bar_w);
                last_touch_pressed = 1;
            }
        } else {
            last_touch_pressed = 0;
        }
    } else {
        last_touch_pressed = 0;
    }

    /* 忽略 result 参数（暂时未使用） */
    (void)result;
}

/**
 * @brief  ?????????(???????)
 */
void C_APP_UI_ResetHistory(void)
{
    temp_history_idx = 0;
    temp_history_count = 0;
    pt100_history_idx = 0;
    pt100_history_count = 0;
}

/*==============================================================================
 * UI 初始化
 *============================================================================*/

/**
 * @brief  初始化 UI 模块：IIC 初始化并清屏
 */
void C_APP_UI_Init(void)
{
    iic_init();
    lcd_clear(WHITE);
}