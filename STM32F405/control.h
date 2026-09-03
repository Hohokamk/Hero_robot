#pragma once
#include <vector>
#include <cmath>
#include "stm32f4xx.h"
#include "motor.h"
#include "imu.h"
#include"HTmotor.h"
#include "pid.h"

class CONTROL final
{
public:
	uint8_t init_DM = 0;
	Motor* chassis_motor[CHASSIS_MOTOR_NUM]{};
	Motor* pantile_motor[PANTILE_MOTOR_NUM]{};
	Motor* shooter_motor[SHOOTER_MOTOR_NUM]{};
	Motor* supply_motor[SUPPLY_MOTOR_NUM]{};
	
	enum MODE { ROTATION, RESET, SEPARATE, FOLLOW, LOCK, TEST, AUTO } mode;
	struct CHASSIS
	{
		PID chassis_reset{};
		int32_t speedx{}, speedy{}, speedz{};
		
		void Keep_Direction();
		void Mecanum_Resolve(int32_t vx, int32_t vy, int32_t wz);
		void Update();
		float Ramp(float setval, float curval, uint32_t RampSlope);
	};

	struct PANTILE
	{
		enum TYPE { PITCH, YAW };
		float mark_pitch{}, mark_yaw{};
		float base_mark_yaw{};      // YAW基准位置（进入保持模式时的电机角度）
		float base_mark_pitch{};    // PITCH基准位置
		// control.h
		PID pantile_PID[3] = { {0.5f, 0.01f, 0.f}, {0.5f, 0.01f, 0.f}, {0.f, 0.f, 0.f} };
		// 或
		PID keep_PID[2] = { {2.0f, 0.05f, 0.1f, 0.f}, {2.0f, 0.05f, 0.1f, 0.f} };
		float pid_pantile_out_speed{};

		const float sensitivity = 2.5f;
		bool aim = false;
		void Keep_Pantile(float angleKeep, PANTILE::TYPE type, IMU& frameOfReference);
		void Update();

		float set_yaw{};                 // 世界目标YAW，单位：度
		bool yaw_hold_initialized = false;
		void SetYawAbsolute(float target_wrapped);
		/*
 * Yaw速度前馈相关参数。
 *
 * yaw_cmd_ff_k：
 * 将底盘speedz指令转换成Yaw电机目标速度。
 *
 * yaw_gyro_fb_k：
 * 将IMU世界Yaw角速度转换成附加速度补偿。
 *
 * 两个系数的正负方向必须通过实车确认。
 */
		float yaw_cmd_ff_k = -0.02f;
		float yaw_gyro_fb_k = -0.05f;

		float yaw_speed_ff{};
		float yaw_speed_ff_target{};

		float yaw_ff_limit = 300.0f;
		float yaw_ff_filter = 0.25f;
	};

	struct SHOOTER
	{
		float now_bullet_speed = 0.f;

		enum class State { IDLE, SPIN_UP, FEED, PUSH, RETRACT, DONE };

		State state = State::IDLE;
		uint32_t state_time = 0;           // 5ms 一周期，按毫秒累加
		uint32_t bullet_detect_cnt = 0;    // 微动开关消抖计数

		int16_t supply_speed = 2160;       // 拨弹盘转速
		uint32_t feed_timeout = 1000;      // 供弹超时
		uint32_t bullet_detect_threshold = 3; // 连续 3 次(15ms)认为上弹到位
		uint32_t push_timeout = 300;       // 推杆推出保持时间
		uint32_t retract_timeout = 300;    // 推杆收回时间

		bool auto_shoot = false;
		bool openRub = false, supply_bullet = false;
		bool fraction = false;
		bool fullheat_shoot = false;
		bool heat_ulimit = false;
		int16_t shoot_speed = 6000;
		void Update();
	};

	CHASSIS chassis;
	PANTILE pantile;
	SHOOTER shooter;
	
	static int16_t Setrange(const int16_t original, const int16_t range);
	void Control_Pantile(int32_t ch_yaw, int32_t ch_pitch);
	float GetDelta(float delta);
	void Init(std::vector<Motor*> motor);
	void init_dm();

private:

};

extern CONTROL ctrl;
