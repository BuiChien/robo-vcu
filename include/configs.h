#pragma once

#define MAX_UINT16                  0xFFFF
#define ENC_PULSES_PER_REVOLUTION   220
#define GEAR_RATIO                  1  // tỉ lệ truyền động 1:1
#define WHEEL_DIAMETER_MM           65  // Đường kính bánh xe
#define MOTOR_VOLTAGE_MAX           12.0
#define MOTOR_VOLTAGE_LOW_LIMIT     0.6
#define ROBOT_RADIUS_MM             77 

#define SERIAL_RECEIVE_BUFFER       128

#define MOTOR_SPEED_SAMPLE_RATE_MS  10  // Tần số lấy mẫu của động cơ
#define LOW_PASS_FILTER_CUTOFF_HZ   25
#define LOW_PASS_FILTER_SAMPLE_HZ   (1000 / MOTOR_SPEED_SAMPLE_RATE_MS)