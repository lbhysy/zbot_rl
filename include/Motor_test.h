#ifndef __Motor_test__
#define __Motor_test__

#include "RS_motor.h"
#include "Hipnuc_IMU.h"
#include <unistd.h> 
#include <thread>
#include <fstream> 
#include <iomanip> 
#include <iostream>
#include <algorithm>
#include <signal.h>
#include <queue>
#include <mutex>
#include <filesystem> 
// Optional libtorch include; use -DUSE_LIBTORCH OFF when building without LibTorch
#ifdef USE_LIBTORCH
#include <torch/script.h>
#endif

#define PI (3.1415926f)

extern int motor_ID; // 电机ID
extern float init_angle;

class Motor_test 
{
public:
    void Spin();

	Motor_test();

	~Motor_test();

	float getTimestamp();

	void Read_Clear(int dev, int num);

private:

    // ************************************************ 线程标志位 ************************************************ //
	
	bool all_thread_done_;
	bool running_;

	// ************************************************ USB2CAN设备 ************************************************ //
	
	int USB2CAN0_;

	// ************************************************ 初始化参数 ************************************************ //
    Motor_PDControl_Struct Motor_test_PD = {
        .Feedforward_Torque = 0.0f,
		.Tar_Position = 0.0f,
		.Tar_Velocity = 0.0f,
		.Kp = 30.0f,
		.Kd = 2.0f,
    };

	// ************************************************ 接收线程相关变量和成员 ************************************************ //
	
	std::thread _Test_RX_thread;
	void Test_RX_thread();

    int can_dev0_rx_count;
	int can_dev0_rx_count_thread;
    
	uint8_t TEMP_ID = 0; // 统一暂存ID

    RS_Motor_Struct Motor_test_receive;

	// ************************************************ 发送线程相关变量和成员 ************************************************ //
	
	std::thread _Test_TX_thread;
	void Test_TX_thread();

	float TX_angle = 0.0f; // 发送到电机的目标角度（rad）

    // ************************************************ 日志线程相关变量和成员 ************************************************ //

	std::thread _Test_Log_thread;
	void Test_Log_thread();

    struct LogFrame {
        float timestamp = 0.0f;
        float Log_angle = 0.0f;
    };
    LogFrame Log_RX, Log_TX;

    std::queue<LogFrame> tx_log_queue;
	std::queue<LogFrame> rx_log_queue;
    std::mutex rx_log_mutex;  // 用于线程安全
    std::mutex tx_log_mutex;  // 用于线程安全

};

#endif
