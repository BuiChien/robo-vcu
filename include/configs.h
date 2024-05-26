#pragma once

#define MAX_UINT16                  0xFFFF
#define PULSE_PER_REVOLUTION        220
#define WHEEL_DIAMETER_MM           65  // Đường kính bánh xe
#define MOTOR_VOLTAGE_MAX           12.0
#define MOTOR_VOLTAGE_MIN           5
#define ROBOT_DIAMETER_MM           77 

#define RECEIVE_CMD_BUFFER_SIZE     8
#define RECEIVE_CMD_QUEUE_SIZE      8

#define MOTOR_SPEED_SAMPLE_RATE_MS  10  // Tần số lấy mẫu của động cơ
#define LOW_PASS_FILTER_CUTOFF_HZ   25
#define LOW_PASS_FILTER_SAMPLE_HZ   (1000 / MOTOR_SPEED_SAMPLE_RATE_MS)

#define NOTIFY_STATUS_INTERVAL_MS   (100)
