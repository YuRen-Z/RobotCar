#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "car_motion.h"
#include "battery.h"
#include "key.h"
#include "encoder.h"
#include "driver/pulse_cnt.h"

//记录整个赛道时间的
#include "esp_timer.h"

/*
 * =============================================================================
 * 1. 声明全局变量以及宏
 * =============================================================================
 *
 */

static const char *TAG = "RACE";


// --- 速度标定 (单位: RPM 或你PID控制器的目标单位) ---
#define SPEED_STRAIGHT_01  0.6 // 顶部大直线的速度
#define SPEED_STRAIGHT_02  0.6 // 右侧短直线的速度
#define SPEED_STRAIGHT_03  0.6 // 底侧短直线的速度
#define SPEED_STRAIGHT_04  0.6 // 左侧短直线的速度

// --- 右转 150 (R 0.65) ---
#define SPEED_R_150_LINE    0.28 // 线速度 v=w*r 
#define SPEED_R_150_W       -0.8 // 角速度 


// --- 右转 90 (R 0.65) ---
#define SPEED_R_90_LINE    0.28 // 线速度
#define SPEED_R_90_W       -0.8 // 角速度

// --- 左转 60 (R 0.65) ---
#define SPEED_L_60_LINE    0.52 // 线速度
#define SPEED_L_60_W       0.8 // 角速度 (左转为正)

// --- 左转 63.97 (R 0.69 - 0.99) ---
#define SPEED_L_63_LINE    0.792 // 线速度
#define SPEED_L_63_W       0.8 // 角速度

// ---  右转 153.97 (R 0.35 - 0.65) ---
#define SPEED_R_153_LINE    0.28 // 线速度
#define SPEED_R_153_W       -0.8 // 角速度

// --- 距离标定 (单位: 编码器平均脉冲数) ---
#define straight_01               15530  // 3.0m 5300大概是1m 但是由于论查耗时 获取脉冲数本身延迟  导致一般会在大于规定脉冲数时才停止 也就是一般会多跑一会 所以设置要偏小一点点
#define turn_right_150            3749   // r0.65 角速度0.8的情况下执行3749ms表示旋转150° 具体实验测试
#define straight_02               2000  //0.5
#define turn_right_90             2250   //r0.65
#define turn_left_60              1500  //r0.65
#define straight_03               2500  //0.5
#define turn_left_63              1599  //0.69 - 0.99
#define turn_right_153            3850   //0.35-0.65
#define straight_04               2500  //0.5
#define turn_right_90_B           2500  //r0.35-0.65   (避免宏定义冲突，改名)

extern pcnt_unit_handle_t encoder_unit_m1;
extern pcnt_unit_handle_t encoder_unit_m2;
extern pcnt_unit_handle_t encoder_unit_m3;
extern pcnt_unit_handle_t encoder_unit_m4;


int encoder_count_M1 = 0;
int encoder_count_M2 = 0;
int encoder_count_M3 = 0;
int encoder_count_M4 = 0;

double p = 0.1924; //一个脉冲对应的路程(mm)
/*
 * =============================================================================
 * 2. 抽象函数 
 * =============================================================================
 *
 */

// 控制小车运动 三个参数分别是 线速度 左右平移速度 角速度
extern void Motion_Ctrl(float V_x, float V_y, float V_z);


//获得当前脉冲数
int get_current_distance(){
       encoder_count_M1 = Encoder_Get_Count_M1();
       encoder_count_M2 = Encoder_Get_Count_M2();
       encoder_count_M3 = Encoder_Get_Count_M3();
       encoder_count_M4 = Encoder_Get_Count_M4();
       
    int total_pulse = encoder_count_M1 + encoder_count_M2 + encoder_count_M3 + encoder_count_M4;

    return total_pulse / 4;
 };

 void clear_all_encoder(){
	if (encoder_unit_m1 != NULL) {
        pcnt_unit_clear_count(encoder_unit_m1);
        }
        if (encoder_unit_m2 != NULL) {
        pcnt_unit_clear_count(encoder_unit_m2);
        }
        if (encoder_unit_m3 != NULL) {
        pcnt_unit_clear_count(encoder_unit_m3);
        }
        if (encoder_unit_m4 != NULL) {
        pcnt_unit_clear_count(encoder_unit_m4);
        }
    }

//编码器脉冲数重置为零
 void reset_encoder_counts(){
    clear_all_encoder();
    encoder_count_M1 = 0;
    encoder_count_M2 = 0;
    encoder_count_M3 = 0;
    encoder_count_M4 = 0;
    return;
 };

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
    STATE_TURN_RIGHT_90_A,  // 4. 右下角右拐九十度
    STATE_TURN_LEFT_60,     // 5. 右下左拐60度弯
    STATE_STRAIGHT_03,      // 6. 底侧小直线
    STATE_TURN_LEFT_63,     // 7. 左拐63.97度弯
    STATE_TURN_RIGHT_153,   // 8. 右拐153.97度弯
    STATE_STRAIGHT_04,      // 9. 左侧小直线
    STATE_TURN_RIGHT_90_B,  // 10. 右上角右拐九十度
    STATE_STOP              // 11. 停止 
} state;

// FSM 任务
void race_task() {
    state current_state = STATE_READY; 
    
    ESP_LOGI(TAG, "FSM 任务已启动。");

    float voltage = Battery_Get_Voltage();
    ESP_LOGI(TAG, "=====================电池状态=====================");
    ESP_LOGI(TAG, "Voltage:%.2fV", voltage);   
    
    double current_distance = 0.0;
    //统计时间用的变量
    int64_t start_time = 0;
    int64_t end_time = 0;

    while (1) {

        current_distance = get_current_distance();

        ESP_LOGI(TAG,"distance now: %f, State: %d", current_distance, current_state);


           if (Key1_Read_State() == 1 )
        {
           ESP_LOGI(TAG, "MANDATORY STOP");
           Motion_Stop(true);
           reset_encoder_counts();
        }




        // 状态机核心
        switch (current_state) {
            
            case STATE_READY:
                Motion_Ctrl(0,0,0);
                vTaskDelay(pdMS_TO_TICKS(100));
                // 切换进入赛道
                ESP_LOGI(TAG, "比赛开始...");
                ESP_LOGI(TAG, "已经进入第一个直线赛道");
                
                current_state = STATE_STRAIGHT_01;

                start_time = esp_timer_get_time();

                // 先重置，再运动
                reset_encoder_counts();
                Motion_Ctrl(SPEED_STRAIGHT_01, 0, 0);
                break;

            case STATE_STRAIGHT_01:
                if(current_distance >= straight_01){
                    ESP_LOGI(TAG, "完成直线 -> 进入第一个右转150度");
                    current_state = STATE_TURN_RIGHT_150;
                    
                    
                    Motion_Stop(false);             // 1. 先停车
                    reset_encoder_counts();         // 2. 重置编码器
                    vTaskDelay(pdMS_TO_TICKS(100)); // 3. 等待车身稳定 (消除惯性)
                    
                    Motion_Ctrl(SPEED_R_150_LINE, 0, SPEED_R_150_W); // 4. 执行下一段动作
                    vTaskDelay(pdMS_TO_TICKS(turn_right_150)); //旋转多长时间 t = 转的角度/360° * T 这里都是角速度0.8所以T都是9s(设置角速度为0.8但实际很可能是0.7几)
                }
                break;

            case STATE_TURN_RIGHT_150:
                if (true) {
                    ESP_LOGI(TAG, "右转150 -> 右下小直线");
                    current_state = STATE_STRAIGHT_02;
                    
                    Motion_Stop(false);
                    reset_encoder_counts();
                    vTaskDelay(pdMS_TO_TICKS(100));

                    Motion_Ctrl(SPEED_STRAIGHT_02, 0, 0);
                }
                break;

            case STATE_STRAIGHT_02:
                if (current_distance >= straight_02) {
                    ESP_LOGI(TAG, "右下小直线完成 -> 右转90度 ");
                    current_state = STATE_TURN_RIGHT_90_A;

                    Motion_Stop(false);
                    reset_encoder_counts();
                    vTaskDelay(pdMS_TO_TICKS(100));

                    Motion_Ctrl(SPEED_R_90_LINE, 0, SPEED_R_90_W);
                    vTaskDelay(pdMS_TO_TICKS(turn_right_90));
                }
                break;

            case STATE_TURN_RIGHT_90_A:
                if (true) {
                    ESP_LOGI(TAG, "右转90完成 -> 左转60度");
                    current_state = STATE_TURN_LEFT_60;

                    Motion_Stop(false);
                    reset_encoder_counts();
                    vTaskDelay(pdMS_TO_TICKS(100));

                    Motion_Ctrl(SPEED_L_60_LINE, 0, SPEED_L_60_W);
                    vTaskDelay(pdMS_TO_TICKS(turn_left_60));
                }
                break;

            case STATE_TURN_LEFT_60:
                if (true) {
                    ESP_LOGI(TAG, "左转60度 -> 底部小直线");
                    current_state = STATE_STRAIGHT_03;

                    Motion_Stop(false);
                    reset_encoder_counts();
                    vTaskDelay(pdMS_TO_TICKS(100));

                    Motion_Ctrl(SPEED_STRAIGHT_03, 0, 0);
                }
                break;
            
            case STATE_STRAIGHT_03:
                if (current_distance >= straight_03) {
                    ESP_LOGI(TAG, " 底部小直线 -> 左转63.97");
                    current_state = STATE_TURN_LEFT_63;

                    Motion_Stop(false);
                    reset_encoder_counts();
                    vTaskDelay(pdMS_TO_TICKS(100));

                    Motion_Ctrl(SPEED_L_63_LINE, 0, SPEED_L_63_W);
                     vTaskDelay(pdMS_TO_TICKS(turn_left_63));
                }
                break;

            case STATE_TURN_LEFT_63:
                if (true) {
                    ESP_LOGI(TAG, "左转63.97 -> 右转153.97");
                    current_state = STATE_TURN_RIGHT_153;

                    Motion_Stop(false);
                    reset_encoder_counts();
                    vTaskDelay(pdMS_TO_TICKS(100));

                    Motion_Ctrl(SPEED_R_153_LINE, 0, SPEED_R_153_W);
                    vTaskDelay(pdMS_TO_TICKS(turn_right_153));
                }
                break;

            case STATE_TURN_RIGHT_153:
                if (true) {
                    ESP_LOGI(TAG, "右转153完成 -> 左侧小直线");
                    current_state = STATE_STRAIGHT_04;

                    Motion_Stop(false);
                    reset_encoder_counts();
                    vTaskDelay(pdMS_TO_TICKS(100));

                    Motion_Ctrl(SPEED_STRAIGHT_04, 0, 0);
                }
                break;

            case STATE_STRAIGHT_04:
                if (current_distance >= straight_04) {
                    ESP_LOGI(TAG, "左侧直线完成 -> 右转90");
                    current_state = STATE_TURN_RIGHT_90_B;

                    Motion_Stop(false);
                    reset_encoder_counts();
                    vTaskDelay(pdMS_TO_TICKS(100));

                    Motion_Ctrl(SPEED_R_90_LINE, 0, SPEED_R_90_W);
                     vTaskDelay(pdMS_TO_TICKS(turn_right_90_B));
                }
                break;

            case STATE_TURN_RIGHT_90_B:
                if (true) { 
                    ESP_LOGI(TAG, "右转九十度完成 -> 结束状态");
                    current_state = STATE_STOP;

                    Motion_Stop(false);
                    reset_encoder_counts();
                    
                    ESP_LOGI(TAG, "结束");
                    end_time = esp_timer_get_time();
                    

                    // rush rush !!!
                     Motion_Ctrl(0.8, 0, 0);
                     vTaskDelay(pdMS_TO_TICKS(250));
                }
                break;

            case STATE_STOP:
                Motion_Stop(false); // 强制刹车

                // esp_timer_get_time 返回的是微秒(us)，除以 1000000.0 变成秒(s)
                if (start_time != 0 && end_time != 0) {
                    double duration = (double)(end_time - start_time) / 1000000.0;
                    ESP_LOGI(TAG, "=================================");
                    ESP_LOGI(TAG, "  比赛结束!  ");
                    ESP_LOGI(TAG, "  Total Time: %.4f s", duration);
                    ESP_LOGI(TAG, "=================================");
                    
                    // 防止重复打印，清零变量或增加标志位
                    start_time = 0; 
                }
                
                // 这里加一个长延时，防止日志刷屏太快
                vTaskDelay(pdMS_TO_TICKS(1000));

                break;
        }
        
        // 10ms 轮询
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void app_main(void)
{
    // 初始化
    vTaskDelay(pdMS_TO_TICKS(1000));
    Key_Init();
    Battery_Init();
    Motor_Init();

    // --- 启动 FSM 任务 ---
    race_task();
}
