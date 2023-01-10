### ArduPilot UUV Project

## History of Changes
# 1. Adding the Custom MotorMatrix
- **File Changed: AP_Motors_Class.h**

Add another parameter to the motor class in line 77-78, specific to the UUV
```
// UUV specified
MOTOR_FRAME_UUV = 18;
```

- **File Changed: AP_MotorsMatrix.cpp**

Define the motor matrix for the UUV in lines 867-900
```
#if AP_MOTORS_FRAME_UUV_ENABLED
	case MOTOR_FRAME_UUV:
		_frame_class_string = "UUV";
		_mav_type = MAV_TYPE_OCTOROTOR;
		switch (frame_type) {
			case MOTOR_FRAME_TYPE_PLUS:{
				_frame_type_string = "PLUS";
				add_motor_raw(0, 0.0f, 0.0f, -1.0f, 0, 1.0f); // esc1 - motor 1
				add_motor_raw(3, 0.0f, 0.0f, 1.0f, 3, 1.0f); // esc4 - motor 4
				add_motor_raw(1, 0.0f, 0.0f, -1.0f, 1, 1.0f); // esc3 - motor 2
				add_motor_raw(2, 0.0f, 0.0f, 1.0f, 2, 1.0f); // esc6 - motor 3
				// add_motor_raw: (motor_num, roll_fac, pitch_fac, yaw_fac, testing_order, throttle_fac)
				// propulsor : throttle_2
				//add_motor_raw(1, 0.0f, 0.0f, 0.0f, 1, 0.1f); // motor5
				//add_motor_raw(4, 0.0f, 0.0f, 0.0f, 4, 0.1f); // motor6

				// robtis servo(): repsonds to roll and pitch
				add_motor_raw(4, -1.0f, -1.0f, 0.0f, 4, 0.0f); // 9 - motor 5
				add_motor_raw(5, -1.0f, 1.0f, 0.0f, 5, 0.0f); // 11 - motor 6
				add_motor_raw(6, -1.0f, 1.0f, 0.0f, 6, 0.0f); // 13 - motor 7
				add_motor_raw(7, -1.0f, -1.0f, 0.0f, 7, 0.0f); // 15 - motor 8

				success = true;
				break;
			}
			default:
				// dodeca-hexa frame class does not support this frame type
				_frame_type_string = "UNSUPPORTED";
				success = false;
				break;
		}
		break;
#endif // UUV ENABLED
```

- **File Changed: System.cpp**

Define the UUV motor frame for the system in line 451

```
case AP_Motors::MOTOR_FRAME_UUV;
```

# 2. Eliminate the coupling between roll & pitch and throttle & yaw

- **File Changed: AP_MotorsMatrix.cpp**

Line 220-223

```
const float compensation_gain = 1.0f; //get_compensation_gain(); // compensation for battery voltage and altitude
roll_thrust = (_roll_in +_roll_in_ff*0.0f) * compensation_gain;
pitch_thrust = (_pitch_in +_pitch_in_ff*0.0f) * compensation_gain;
yaw_thrust = (_yaw_in + _yaw_in_ff*0.0f) * compensation_gain;
```
Line 335

```
if (i == 0 || i == 1 || i == 2 || i == 3) {
```

Line 396
```
if (i == 0 || i == 1 || i == 2 || i == 3) {
```

- **File Changed: AC_AttitudeControl.cpp**

Line 303-335

```
void AC_AttitudeControl::input_euler_angle_roll_pitch_euler_rate_yaw_uuv(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds)
{
	// Convert from centidegrees on public interface to radians
	float euler_roll_angle = radians(euler_roll_angle_cd * 0.01f);
	float euler_pitch_angle = radians(euler_pitch_angle_cd * 0.01f);
	float euler_yaw_rate = radians(euler_yaw_rate_cds * 0.01f);

	//const AP_AHRS &ahrs = AP::ahrs();
	//_euler_angle_target.x = ahrs.roll;
	//_euler_angle_target.y = ahrs.pitch;
	//_euler_angle_target.z = ahrs.yaw;

	Quaternion attitude_body;
    _ahrs.get_quat_body_to_ned(attitude_body);
	attitude_body.to_euler(_euler_angle_target.x, _euler_angle_target.y, _euler_angle_target.z);

	const Vector3f euler_accel = {get_accel_roll_max_radss(), get_accel_pitch_max_radss(), get_accel_yaw_max_radss()};

	 _euler_rate_target.x = input_shaping_angle(wrap_PI(euler_roll_angle - _euler_angle_target.x), _input_tc, euler_accel.x, _euler_rate_target.x, _dt);
	 _euler_rate_target.y = input_shaping_angle(wrap_PI(euler_pitch_angle - _euler_angle_target.y), _input_tc, euler_accel.y, _euler_rate_target.y, _dt);
	 _euler_rate_target.z = input_shaping_ang_vel(_euler_rate_target.z, euler_yaw_rate, euler_accel.z, _dt);

	 _ang_vel_body = _euler_rate_target;
	 ang_vel_limit(_ang_vel_body, radians(_ang_vel_roll_max), radians(_ang_vel_pitch_max), radians(_ang_vel_yaw_max));
	 _feedforward_scalar = 1.0f;

	 Quaternion attitude_target_update;
	 attitude_target_update.from_axis_angle(Vector3f{_euler_rate_target.x*_dt, _euler_rate_target.y*_dt, _euler_rate_target.z*_dt});
	 _attitude_target = _attitude_target*attitude_target_update;
	 _attitude_target.normalize();
	 _attitude_ang_error = attitude_body.inverse()*_attitude_target;

}
```

- **File Changed: AC_AttitudeControl.h**

Lines 157-158

```
// UUV customized
    virtual void input_euler_angle_roll_pitch_euler_rate_yaw_uuv(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds);
```
# 3. Disable the pwm output for motors when disarmed

- **File Changed: AP_MotorsMatrix.cpp**

Line 146

```
AP_GROUPINFO("SAFE_DISARM", 23, AP_MotorsMulticopter, _disarm_disable_pwm, 1), // change 0 to 1
```

In Mission Planner, set MOT_SAFE_DISARM to 1

# 4. Set the output pwm to be in the middle when started up

- **File Changed: AP_Multicopter.cpp**

Line 440

```
pwm_output = get_pwm_output_min() + 500.0f; // add 500.0f
```

- **File Changed: AP_MotorMatrix.cpp

Line 162-167

```
if (i == 0 || i == 1 || i == 2 || i == 3){
    set_actuator_with_slew(_actuator[i], 0.5f);
}
else {
    set_actuator_with_slew(_actuator[i], 0.0f);//actuator_spin_up_to_ground_idle());
}
```

# 5. Add trim for Rob servos (Motor 5-8) using servo trim input in Mission Planner

- **File Changed: AP_MotorsMatrix
Line 19

```
#include <SRV_Channel/SRV_Channel.h>
```

Line 171-178

```
if (motor_enabled[i]) {
    if (i == 4 || i == 5 || i == 6 || i == 7){
    _actuator[i] = _thrust_rpyt_out[i]; // scale combined pitch and roll inputs to +/- 2.0
}
else {
    set_actuator_with_slew(_actuator[i], thrust_to_actuator(_thrust_rpyt_out[i]));
}
}
```

Line 183-215

```
// convert output to PWM and send to each motor
    for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
        	// motor 5
        	if (i == 4){
        		int16_t value;
        		value = SRV_Channels::return_channel(SRV_Channel::k_motor5)->get_trim() + 500.0f * _actuator[i];
        		rc_write(i, value);
        	}
        	// motor 6
        	else if (i == 5){
				int16_t value;
				value = SRV_Channels::return_channel(SRV_Channel::k_motor6)->get_trim() + 500.0f * _actuator[i];
				rc_write(i, value);
        	}
        	// motor 7
        	else if (i == 6){
				int16_t value;
				value = SRV_Channels::return_channel(SRV_Channel::k_motor7)->get_trim() + 500.0f * _actuator[i];
				rc_write(i, value);
			}
        	// motor 8
        	else if (i == 7){
				int16_t value;
				value = SRV_Channels::return_channel(SRV_Channel::k_motor8)->get_trim() + 500.0f * _actuator[i];
				rc_write(i, value);
			}
        	// others
        	else {
        		rc_write(i, output_to_pwm(_actuator[i]));
        	}
        }
    }
```

- **File Change: SRV_Channels.h

Line 468-469

```
// return channel that a function is assigned to
    static SRV_Channel *return_channel(SRV_Channel::Aux_servo_function_t function);
```

