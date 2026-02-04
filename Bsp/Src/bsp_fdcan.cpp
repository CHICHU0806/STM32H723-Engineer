//
// Created by 20852 on 2026/2/4.
//

#include "bsp_fdcan.h"

DJI_motor_info djimotor1;

void bsp_fdcan::bsp_fdcan_init()
{
    static bsp_fdcan fdcan;

    // 1. 配置 FDCAN 过滤器
    fdcan.BSP_FDCAN_FilterConfig();

    // 2. 启动 FDCAN 外设
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }
    // 如果有其他 CAN 外设，也在这里启动

    // 3. 激活 FDCAN 接收中断 (当 FIFO0 中有新消息时触发)
    if (HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0) != HAL_OK)
    {
        Error_Handler(); // 激活中断失败
    }
    // 如果有其他 CAN 外设，也在这里激活中断
}

void bsp_fdcan::BSP_FDCAN_FilterConfig()
{
    FDCAN_FilterTypeDef filter;

    filter.IdType       = FDCAN_STANDARD_ID;
    filter.FilterIndex  = 0;
    filter.FilterType   = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

    filter.FilterID1    = 0x000;
    filter.FilterID2    = 0x000;

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_FILTER_REMOTE,
        FDCAN_FILTER_REMOTE
    );
}

HAL_StatusTypeDef bsp_fdcan::BSP_FDCAN_DJIMotorCmd(int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4){
    FDCAN_TxHeaderTypeDef TxHeader = {0};
    uint8_t TxData[8];

    TxHeader.Identifier          = 0x200;
    TxHeader.IdType              = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType         = FDCAN_DATA_FRAME;
    TxHeader.DataLength          = FDCAN_DLC_BYTES_8;

    TxHeader.FDFormat            = FDCAN_CLASSIC_CAN;
    TxHeader.BitRateSwitch       = FDCAN_BRS_OFF;

    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;

    TxData[0] = motor1 >> 8;
    TxData[1] = motor1;
    TxData[2] = motor2 >> 8;
    TxData[3] = motor2;
    TxData[4] = motor3 >> 8;
    TxData[5] = motor3;
    TxData[6] = motor4 >> 8;
    TxData[7] = motor4;

    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}

// FDCAN 接收中断回调函数
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    // 判断是不是“FIFO0 有新消息”中断
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
            /* ================= FDCAN1处理 ================= */
            if (hfdcan->Instance == FDCAN1)
            {
                switch (RxHeader.Identifier)
                {
                    case 0x204://此处仅接收了id为0x204电机的报文
                    {
                        djimotor1.rotor_angle    = ((RxData[0] << 8) | RxData[1]);
                        djimotor1.rotor_speed    = ((RxData[2] << 8) | RxData[3]);
                        djimotor1.torque_current = ((RxData[4] << 8) | RxData[5]);
                        djimotor1.temp           =   RxData[6];
                        break;
                    }
                    default: break;
                }
            }
        }
    }
}

// C接口封装
extern "C" {
    static bsp_fdcan fdcan;

    void BSP_FDCAN_Init() {
        fdcan.bsp_fdcan_init();
    }

    HAL_StatusTypeDef bsp_fdcan_djimotorcmd(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4) {
        return fdcan.BSP_FDCAN_DJIMotorCmd(motor1, motor2, motor3, motor4);
    }
}