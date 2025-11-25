#include "car_motion.h"


#include "stdio.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "motor.h"


car_motion_t micro_car;

// // 线速度和角速度
// static float line_v = 0;
// static float angular_v = 0;

static float speed_L1_setup = 0;
static float speed_L2_setup = 0;
static float speed_R1_setup = 0;
static float speed_R2_setup = 0;


// 小车停止 Car stop
void Motion_Stop(uint8_t brake)
{
    Motor_Stop(brake);
}

// 控制小车运动, V_x表示控制线速度[-1.0, 1.0]，V_y无效，V_z表示控制角速度[-5.0, 5.0]。
// Control car motion, V_x control line speed [-1.0, 1.0], V_y is invalid, V_z control angular speed [-5.0, 5.0]
void Motion_Ctrl(float V_x, float V_y, float V_z)
{
    // line_v = V_x;
    // angular_v = V_z;
    // speed_L1_setup = line_v - angular_v * ROBOT_APB;
    // speed_L2_setup = line_v - angular_v * ROBOT_APB;
    // speed_R1_setup = line_v + angular_v * ROBOT_APB;
    // speed_R2_setup = line_v + angular_v * ROBOT_APB;
    // Motor_Set_Speed(speed_L1_setup, speed_L2_setup, speed_R1_setup, speed_R2_setup);
    // // 限制输入范围 (可选，为了安全)
    
    // V_x: [-1.0, 1.0], V_y: [-1.0, 1.0], V_z: [-5.0, 5.0]

    float angular_component = V_z * ROBOT_APB;

    // 麦克纳姆轮运动学公式
    // 注意：V_y 的加减号取决于你的电机安装方向和轮子滚轮方向（O型安装 vs X型安装）
    // 以下是标准的 O型长方形底盘 且 想要向左平移时的公式 (视 V_y 正负而定)
    
    // 左前轮 (L1) = Vx - Vy - Vz
    speed_L1_setup = V_x - V_y - angular_component;
    
    // 左后轮 (L2) = Vx + Vy - Vz
    speed_L2_setup = V_x + V_y - angular_component;
    
    // 右前轮 (R1) = Vx + Vy + Vz
    speed_R1_setup = V_x + V_y + angular_component;
    
    // 右后轮 (R2) = Vx - Vy + Vz
    speed_R2_setup = V_x - V_y + angular_component;

    // 发送给电机
    Motor_Set_Speed(speed_L1_setup, speed_L2_setup, speed_R1_setup, speed_R2_setup);
}

// 获取小车运动的速度
// Get the speed of the car's motion
void Motion_Get_Speed(car_motion_t* car)
{
    float speed_m1 = 0, speed_m2 = 0, speed_m3 = 0, speed_m4 = 0;
    Motor_Get_Speed(&speed_m1, &speed_m2, &speed_m3, &speed_m4);

    car->Vx = (speed_m1 + speed_m2 + speed_m3 + speed_m4) / 4;
    car->Vy = 0;
    car->Wz = -(speed_m1 + speed_m2 - speed_m3 - speed_m4) / 4.0f / ROBOT_APB;
    if(car->Wz == 0) car->Wz = 0;
}

// 控制小车的运动状态
// Control the motion state of the car
void Motion_Ctrl_State(uint8_t state, float speed)
{
    if (speed < 0) speed = -speed;
    if (speed > 1.0) speed = 1.0;
    switch (state)
    {
    case MOTION_STOP:
        Motion_Stop(STOP_COAST);
        break;
    case MOTION_RUN:
        Motion_Ctrl(speed, 0, 0);
        break;
    case MOTION_BACK:
        Motion_Ctrl(-speed, 0, 0);
        break;
    case MOTION_LEFT:
        Motion_Ctrl(speed, 0, speed*ROBOT_SPIN_SCALE);
        break;
    case MOTION_RIGHT:
        Motion_Ctrl(speed, 0, -speed*ROBOT_SPIN_SCALE);
        break;
    case MOTION_SPIN_LEFT:
        Motion_Ctrl(0, 0, speed*ROBOT_SPIN_SCALE);
        break;
    case MOTION_SPIN_RIGHT:
        Motion_Ctrl(0, 0, -speed*ROBOT_SPIN_SCALE);
        break;
    case MOTION_BRAKE:
        Motion_Stop(STOP_BRAKE);
        break;
    default:
        break;
    }
}
