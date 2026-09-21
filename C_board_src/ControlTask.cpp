/*
 * @Author: Guevara-666 2662443905@qq.com
 * @Date: 2026-09-19 11:34:01
 * @LastEditors: Guevara-666 2662443905@qq.com
 * @LastEditTime: 2026-09-21 21:58:15
 * @FilePath: \sec\C_board_src\ControlTask.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "ControlTask.h"
#include "Gm6020.h"
#include "Pid.h"
#include "can.h" // 包含 HAL CAN 头文件
#include "tim.h" // 包含 HAL TIM 头文件
#include <math.h>

// 实例化电机对象 (拨码开关ID设为2)
Gm6020 motor(2);

// 实例化 PID 控制器 (参数需根据实际调试调整)
// Kp, Ki, Kd, 输出上限, 输出下限
Pid speed_pid(20.0f, 0.5f, 0.0f, 16384.0f, -16384.0f);

// 位置控制
//Pid pos_pid(250.0f, 1.0f, 0.01f, 16384.0f, -16384.0f);


// 正弦相关变量
float time_s = 0.0f;
const float amplitude_rpm = 100.0f; // 正弦速度幅值 100 rpm
const float frequency_hz  = 0.5f;   // 频率 0.5 Hz (周期2秒)

/*
// 正弦位置控制相关变量
float time_s = 0.0f;
const float amplitude_rad = 1.0f;    // 正弦位置幅值
const float frequency_hz  = 0.5f;    // 0.5 Hz
*/

extern "C" {

void ControlTaskInit(void) {
    // 1. 配置 CAN1 过滤器 (接收 0x205 ~ 0x20B 的反馈报文)
    CAN_FilterTypeDef can_filter;
    can_filter.FilterActivation = ENABLE;
    can_filter.FilterBank = 0;
    can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter.FilterIdHigh = 0x205 << 5; // 基础ID
    can_filter.FilterIdLow = 0;
    can_filter.FilterMaskIdHigh = 0x7F0 << 5; // 掩码，0x205~0x20B
    can_filter.FilterMaskIdLow = 0;
    can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter.SlaveStartFilterBank = 14; // 双CAN时使用

    if (HAL_CAN_ConfigFilter(&hcan1, &can_filter) != HAL_OK) {
        // 初始化失败处理
        Error_Handler();
    }

    // 2. 启动 CAN
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        Error_Handler();
    }
    // 使能 CAN 接收中断
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler();
    }

    // 3. 启动 1kHz 控制定时器 (TIM6)
    HAL_TIM_Base_Start_IT(&htim6);
}

void MainTask(void) {
    // 1kHz 周期执行
    time_s += 0.001f; // 累加时间

    static float target_rpm = 0.0f;
    static float current_rpm = 0.0f;
    static float voltage_out = 0.0f;

    // 计算目标正弦速度 (rpm)
    target_rpm = amplitude_rpm * sinf(2.0f * 3.1415926f * frequency_hz * time_s);

    // 获取当前电机速度 (rpm)
    current_rpm = (float)motor.velRpm();

    // PID 计算输出电压 (-25000 ~ 25000) -> (-16384 ~ 16384)(电流控制)
    voltage_out = speed_pid.calc(target_rpm, current_rpm);
    
    /*
    static float target_rad = 0.0f;
    static float current_rad = 0.0f;
    static float voltage_out = 0.0f;


    // 1. 读原始反馈并滤波
    static float filtered_angle = 0.0f;
    const float alpha = 0.4f;

    // 首次运行时，直接用原始值初始化滤波值，避免从 0 爬升
    if (filtered_angle == 0.0f) {
        filtered_angle = 57.3f * motor.angle();
    }

    filtered_angle = alpha * (57.3f * motor.angle()) + (1.0f - alpha) * filtered_angle;
    current_rad = filtered_angle;

    // 2. 计算正弦目标
    float sine_target = 57.3f * amplitude_rad * sinf(2.0f * 3.1415926f * frequency_hz * time_s);

    // 3. 启动斜坡：目标从当前反馈值平滑过渡到正弦目标
    static float startup_ramp = 0.0f;
    if (startup_ramp < 1.0f) {
        startup_ramp += 0.0005f;
        if (startup_ramp > 1.0f) startup_ramp = 1.0f;
    }

    target_rad = current_rad * (1.0f - startup_ramp) + sine_target * startup_ramp;
    

    // 4. PID
    voltage_out = pos_pid.calc(target_rad, current_rad);
    */

    // 设置电机电压并发送 CAN 报文
    motor.setVoltage((int16_t)voltage_out);

    // 准备 CAN 发送数据
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8] = {0};
    uint32_t tx_mailbox;

    tx_header.StdId = motor.txId();
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    // 将电压值编码到 8 字节数据中
    motor.encode(tx_data);

    // 发送 CAN 报文
    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);
}

void CanFeedbackCallback(uint32_t std_id, const uint8_t *data) {
    // 判断是否为电机 2 的反馈报文 (0x206)
    if (std_id == motor.rxId()) {
        motor.decode(data);
    }
}

}
