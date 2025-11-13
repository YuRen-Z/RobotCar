#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "car_motion.h"
#include "battery.h"
#include "key.h"


static const char *TAG = "RACE";


// --- 速度标定 (单位: RPM 或你PID控制器的目标单位) ---
#define SPEED_STRAIGHT_01  0 // 顶部大直线的速度
#define SPEED_STRAIGHT_02  0 // 右侧短直线的速度
#define SPEED_STRAIGHT_03  0 // 底侧短直线的速度
#define SPEED_STRAIGHT_04  0 // 左侧短直线的速度

// --- 右转 150 (R 0.65) ---
#define SPEED_R_150_OUTER   0 // 外轮 (左轮)
#define SPEED_R_150_INNER   0 // 内轮 (右轮)
#define SPEED_R_150_LINE    0 // 线速度
#define SPEED_R_150_W       0 // 角速度


// --- 右转 90 (R 0.65) ---
#define SPEED_R_90_OUTER    0//外轮（左轮）
#define SPEED_R_90_INNER    0//内轮（右轮）
#define SPEED_R_90_LINE    0 // 线速度
#define SPEED_R_90_W       0 // 角速度

// --- 左转 60 (R 0.65) ---
#define SPEED_L_90_OUTER    0//外轮（右轮）
#define SPEED_L_90_INNER    0//内轮（左轮）
#define SPEED_L_60_LINE    0 // 线速度
#define SPEED_L_60_W       0 // 角速度

// --- 左转 63.97 (R 0.69 - 0.99) ---
#define SPEED_L_63_OUTER  0 // 外轮 (右轮)
#define SPEED_L_63_INNER  0 // 内轮 (左轮)
#define SPEED_L_63_LINE    0 // 线速度
#define SPEED_L_63_W       0 // 角速度

// ---  右转 153.97 (R 0.35 - 0.65) ---
#define SPEED_R_153_OUTER  0 // 外轮 (左轮)
#define SPEED_R_153_INNER  0 // 内轮 (右轮)
#define SPEED_R_153_LINE    0 // 线速度
#define SPEED_R_153_W       0 // 角速度

// --- 右转 90 (R 0.35-0.65) ---
#define SPEED_R_90_OUTER    0//外轮（左轮）
#define SPEED_R_90_INNER    0//内轮（右轮）
#define SPEED_R_90_LINE    0 // 线速度
#define SPEED_R_90_W       0 // 角速度

// --- 距离标定 (单位: 编码器平均脉冲数) ---
#define straight_01               50000  // 3.0m
#define turn_right_150            8000   // r0.65
#define straight_02               15000  //0.5
#define turn_right_90             9000   //r0.65
#define turn_left_60              11000  //r0.65
#define straight_03               10000  //0.5
#define turn_left_63              14000  //0.69 - 0.99
#define turn_right_153            7000   //0.35-0.65
#define straight_04               10000  //0.5
#define turn_right_90             9000   //r0.35-0.65

/*
 * =============================================================================
 * 2. 抽象函数 
 * =============================================================================
 *
 */

// 控制小车运动, V_x表示控制线速度[-1.0, 1.0]，V_y无效，V_z表示控制角速度[-5.0, 5.0]。
extern void Motion_Ctrl(float V_x, float V_y, float V_z);

//获得当前脉冲数
extern int64_t get_current_distance();

//编码器脉冲数重置为零
extern void reset_encoder_counts();

/*
 * =============================================================================
 * 3. 有限状态机 (FINITE STATE MACHINE)
 * =============================================================================
 */

// FSM 的所有状态
typedef enum {
    STATE_READY,            // 0. 准备阶段，等待发车
    STATE_STRAIGHT_01,      // 1. 第一部分直线
    STATE_TURN_RIGHT_150,   // 2. 右上150度弯
    STATE_STRAIGHT_02,      // 3. 右下侧小直线
    STATE_TURN_RIGHT_90_A,    // 4. 右下角右拐九十度
    STATE_TURN_LEFT_60,     // 5. 右下左拐60度弯
    STATE_STRAIGHT_03,      // 6. 底侧小直线
    STATE_TURN_LEFT_63,     // 7. 左拐63.97度弯
    STATE_TURN_RIGHT_153,   // 8. 右拐153.97度弯
    STATE_STRAIGHT_04,      // 9. 左侧小直线
    STATE_TURN_RIGHT_90_B,    //10. 右上角右拐九十度
    STATE_STOP              //11. 停止 
} state;

// FSM 任务
void race_task() {
    state current_state = STATE_READY; // 自动开始
    int64_t current_distance = 0;
    

    ESP_LOGI(TAG, "FSM 任务已启动。");

    while (1) {

        int64_t current_distance = get_current_distance();

        // 状态机核心
        switch (current_state) {
            
            case STATE_READY:
                Motion_Ctrl(0,0,0);
                vTaskDelay(pdMS_TO_TICKS(100));
                //切换进入赛道
                ESP_LOGI(TAG, "比赛开始...");
                ESP_LOGI(TAG, "已经进入第一个直线赛道");
                current_state =STATE_STRAIGHT_01;
                reset_encoder_counts();
                Motion_Ctrl(SPEED_STRAIGHT_01,0,0);
                break;

            case STATE_STRAIGHT_01:
                if(current_distance >= straight_01){
                    //已经跑够第一个部分了 进入下一个部分
                    ESP_LOGI(TAG, "完成直线 -> 进入第一个右转150度");
                    current_state =STATE_TURN_RIGHT_150;
                    reset_encoder_counts();
                    Motion_Ctrl(SPEED_R_150_LINE,0,SPEED_R_150_W);
                }
                break;

            case STATE_TURN_RIGHT_150:
                if (current_distance >= turn_right_150) {
                    ESP_LOGI(TAG, "右转150 -> 右下小直线");
                    current_state = STATE_STRAIGHT_02;
                    reset_encoder_counts();
                    Motion_Ctrl(SPEED_STRAIGHT_02,0,0);
                }
                break;

            case STATE_STRAIGHT_02:
                if (current_distance >= straight_02) {
                    ESP_LOGI(TAG, "右下小直线完成 -> 右转90度 ");
                    current_state = STATE_TURN_RIGHT_90;
                    reset_encoder_counts();
                    Motion_Ctrl(SPEED_R_90_LINE,0,SPEED_R_90_W);
                }
                break;

            case STATE_TURN_RIGHT_90_A:
                if (current_distance >= turn_right_90) {
                    ESP_LOGI(TAG, "右转90完成 -> 左转60度");
                    current_state = STATE_TURN_LEFT_60;
                    reset_encoder_counts();
                    Motion_Ctrl(SPEED_L_60_LINE,0,SPEED_L_60_W);
                }
                break;

            case STATE_TURN_LEFT_60:
                if (current_distance >= turn_left_60) {
                    ESP_LOGI(TAG, "左转60度 -> 底部小直线");
                    current_state = STATE_STRAIGHT_03;
                    reset_encoder_counts();
                    Motion_Ctrl(SPEED_STRAIGHT_03,0,0);
                }
                break;
            
            case STATE_STRAIGHT_03:
                if (current_distance >= straight_03) {
                    ESP_LOGI(TAG, " 底部小直线 -> 左转63.97");
                    current_state = STATE_TURN_LEFT_63;
                    reset_encoder_counts();
                    Motion_Ctrl(SPEED_L_63_LINE,0,SPEED_L_63_W);
                }
                break;

            case STATE_TURN_LEFT_63:
                if (current_distance >= turn_left_63) {
                    ESP_LOGI(TAG, "左转63.97 -> 右转153.97");
                    current_state = STATE_TURN_RIGHT_153;
                    reset_encoder_counts();
                    Motion_Ctrl(SPEED_R_153_LINE,0,SPEED_R_153_W);
                }
                break;

            case STATE_TURN_RIGHT_153:
                if (current_distance >= turn_right_153) {
                    ESP_LOGI(TAG, "右转153完成 -> 左侧小直线");
                    current_state = STATE_STRAIGHT_04;
                    reset_encoder_counts();
                    Motion_Ctrl(SPEED_STRAIGHT_04,0,0);
                }
                break;

            case STATE_STRAIGHT_04:
                if (current_distance >= straight_04) {
                    ESP_LOGI(TAG, "左侧直线完成 -> 右转90");
                    current_state = STATE_TURN_RIGHT_90;
                    reset_encoder_counts();
                    Motion_Ctrl(SPEED_R_90_LINE,0,SPEED_R_90_W);
                }
                break;

            case STATE_TURN_RIGHT_90_B:
                if (current_distance >= turn_right_90) {
                    ESP_LOGI(TAG, "右转九十度完成 -> 结束状态");
                    current_state = STATE_STOP;
                    Motion_Ctrl(0,0,0);
                }
                break;

            case STATE_STOP:
                ESP_LOGI(TAG, "结束");
                break;
        }
        // FSM 轮询间隔
        // 10ms-20ms 是一个很好的值。
        // 太快会浪费CPU，太慢会让你在高速时冲过头 (overshoot)
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void app_main(void)
{
    
    vTaskDelay(pdMS_TO_TICKS(1000));

    Key_Init();
    Battery_Init();
    Motor_Init();
    race_task();

    float voltage = Battery_Get_Voltage();
    ESP_LOGI(TAG, "=====================电池状态=====================");
    ESP_LOGI(TAG, "Voltage:%.2fV", voltage);


    while (1)
    {
        if (Key1_Read_State() == KEY_STATE_PRESS)
        {
           ESP_LOGI(TAG, "KEY 1 PRESS");
           Motion_Ctrl(0,0,0);
        }
    }
    
}
