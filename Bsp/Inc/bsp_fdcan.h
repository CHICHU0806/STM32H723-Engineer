//
// Created by 20852 on 2026/2/4.
//

#ifndef H723TEST_BSP_FDCAN_H
#define H723TEST_BSP_FDCAN_H

#pragma once
#include "fdcan.h"

//大疆系列电机反馈信息
typedef struct {
    int16_t rotor_angle;      // 电机转子角度
    int16_t rotor_speed;      // 电机转子速度
    int16_t torque_current;   // 电机转矩电流
    int8_t  temp;             // 电机温度

    uint16_t last_angle;      // 上次角度
    int16_t  total_angle;     // 累计角度
    int32_t  round_count;     // 圈数计数
    uint8_t  inited;          // 是否初始化标志
} DJI_motor_info;

//达妙系列电机反馈信息
typedef struct {
    uint8_t  id;           // 电机 ID (0~15)
    uint8_t  err;          // 电机错误码

    uint16_t pos_raw;      // 原始位置（12bit/16bit根据型号）
    uint16_t vel_raw;      // 原始速度（12bit）
    uint16_t torque_raw;   // 原始力矩（12bit）

    uint8_t  temp_mos;     // MOS温度

    float pos;             // 物理量（可选，自动算）
    float vel;             // 物理量（可选）
    float torque;          // 物理量（可选）

} DM_motor_info;

//瓴控系列电机反馈信息
typedef struct {
    uint16_t rotor_angle;      // 电机转子角度
    int16_t rotor_speed;      // 电机转子速度
    int16_t torque_current;   // 电机转矩电流
    int8_t  temp;             // 电机温度
} LK_motor_info;

#ifdef __cplusplus
class bsp_fdcan {
public:
    void bsp_fdcan_init();
    void BSP_FDCAN_FilterConfig();

    //DJI Motor CAN发送函数接口
    HAL_StatusTypeDef BSP_FDCAN_DJIMotorCmd(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

    void BSP_FDCAN_Init();

    //DJI Motor CAN发送函数接口
    HAL_StatusTypeDef bsp_fdcan_djimotorcmd(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
#ifdef __cplusplus
}
#endif


#endif //H723TEST_BSP_FDCAN_H