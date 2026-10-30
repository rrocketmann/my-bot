#include "main.h"
#include "auton.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

// Motor encoders already include the cartridge. gear_ratio is only the
// external reduction from the motor output to the wheel (wheel turns / motor turns).
// 36 driving a 60 tooth gear is 36.0 / 60.0. Direct drive is 1.0.
constexpr double kWheelDiameter = 3.25;
constexpr double kGearRatio = 1.0;
// Distance between the left and right wheel contact patches, in inches.
// If turns overshoot, raise this. If turns come up short, lower it.
constexpr double kTrackWidth = 12.0;

constexpr double kPi = 3.14159265358979323846;

// Error is inches or radians. Output is motor voltage, -127 to 127.
constexpr double kDistanceP = 6.0;
constexpr double kDistanceD = 2.0;
constexpr double kHeadingP = 90.0;
constexpr double kHeadingD = 12.0;
constexpr double kStaticVoltage = 10.0;

constexpr double kSettleDistance = 1.0;
constexpr int kSettleTimeMs = 200;

extern pros::MotorGroup left_drive;
extern pros::MotorGroup right_drive;
extern pros::MotorGroup dr4b;
extern pros::adi::DigitalOut claw;

Pose robot_pose{0.0, 0.0, 0.0};
double prev_left_inches = 0.0;
double prev_right_inches = 0.0;
bool encoders_ready = false;

double wrap_angle(double angle) {
	return std::remainder(angle, 2.0 * kPi);
}

double side_inches(pros::MotorGroup& group) {
	const std::vector<double> positions = group.get_position_all();
	double sum = 0.0;
	int count = 0;
	for (double degrees : positions) {
		if (!std::isfinite(degrees)) {
			continue;
		}
		sum += degrees;
		++count;
	}
	if (count == 0) {
		return 0.0;
	}
	const double motor_turns = (sum / count) / 360.0;
	return motor_turns * kGearRatio * kPi * kWheelDiameter;
}

void sync_encoders() {
	prev_left_inches = side_inches(left_drive);
	prev_right_inches = side_inches(right_drive);
	encoders_ready = true;
}

void position_init() {
	left_drive.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
	right_drive.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
	sync_encoders();
	robot_pose = {0.0, 0.0, 0.0};
}

void position_set(Pose pose) {
	robot_pose = pose;
	robot_pose.theta = wrap_angle(robot_pose.theta);
	sync_encoders();
}

Pose position_get() {
	return robot_pose;
}

void position_update() {
	if (!encoders_ready) {
		sync_encoders();
	}

	const double left_inches = side_inches(left_drive);
	const double right_inches = side_inches(right_drive);
	const double delta_left = left_inches - prev_left_inches;
	const double delta_right = right_inches - prev_right_inches;
	prev_left_inches = left_inches;
	prev_right_inches = right_inches;

	const double delta_center = (delta_left + delta_right) / 2.0;
	const double delta_theta = (delta_right - delta_left) / kTrackWidth;
	const double mid_theta = robot_pose.theta + delta_theta / 2.0;

	robot_pose.x += delta_center * std::cos(mid_theta);
	robot_pose.y += delta_center * std::sin(mid_theta);
	robot_pose.theta = wrap_angle(robot_pose.theta + delta_theta);

	pros::lcd::print(2, "x %.1f  y %.1f  h %.1f", robot_pose.x, robot_pose.y,
	                 robot_pose.theta * 180.0 / kPi);
}

int clamp_voltage(double voltage) {
	if (voltage > 127.0) {
		return 127;
	}
	if (voltage < -127.0) {
		return -127;
	}
	return static_cast<int>(std::lround(voltage));
}

struct Pid {
	double kP;
	double kD;
	double prev_error = 0.0;
	bool has_prev = false;

	double update(double error, double dt) {
		double derivative = 0.0;
		if (has_prev && dt > 0.0) {
			derivative = (error - prev_error) / dt;
		}
		prev_error = error;
		has_prev = true;
		return kP * error + kD * derivative;
	}

	void reset() {
		prev_error = 0.0;
		has_prev = false;
	}
};

void drive_to_point(double x, double y, int timeout_ms) {
	left_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
	right_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);

	Pid distance_pid{kDistanceP, kDistanceD};
	Pid heading_pid{kHeadingP, kHeadingD};

	const std::uint32_t start = pros::millis();
	std::uint32_t prev_time = start;
	std::uint32_t settled_since = 0;

	while (pros::millis() - start < static_cast<std::uint32_t>(timeout_ms)) {
		const std::uint32_t now = pros::millis();
		const double dt = (now - prev_time) / 1000.0;
		prev_time = now;

		position_update();
		const Pose pose = robot_pose;

		const double dx = x - pose.x;
		const double dy = y - pose.y;
		const double distance = std::hypot(dx, dy);
		const double heading_to_target = std::atan2(dy, dx);

		double heading_error = wrap_angle(heading_to_target - pose.theta);
		double direction = 1.0;
		if (std::abs(heading_error) > kPi / 2.0) {
			direction = -1.0;
			heading_error = wrap_angle(heading_to_target + kPi - pose.theta);
		}

		if (distance < kSettleDistance) {
			if (settled_since == 0) {
				settled_since = now;
			}
			if (now - settled_since >= static_cast<std::uint32_t>(kSettleTimeMs)) {
				break;
			}
		} else {
			settled_since = 0;
		}

		double linear = direction * distance_pid.update(distance, dt);
		// Full forward power only once the robot is facing its travel direction.
		linear *= std::cos(heading_error);
		if (distance > kSettleDistance && std::abs(linear) > 1.0) {
			linear += std::copysign(kStaticVoltage, linear);
		}
		const double angular = heading_pid.update(heading_error, dt);

		double left_voltage = linear - angular;
		double right_voltage = linear + angular;
		const double peak = std::max(std::abs(left_voltage), std::abs(right_voltage));
		if (peak > 127.0) {
			left_voltage *= 127.0 / peak;
			right_voltage *= 127.0 / peak;
		}

		left_drive.move(clamp_voltage(left_voltage));
		right_drive.move(clamp_voltage(right_voltage));
		pros::delay(10);
	}

	left_drive.move(0);
	right_drive.move(0);
}

void run_dr4b(int voltage, int duration_ms) {
	dr4b.move(voltage);
	pros::delay(duration_ms);
	dr4b.move(0);
}

void autonomous() {
	position_set({0.0, 0.0, 0.0});
	dr4b.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

	// Short straight move, then a timed lift cycle and a claw cycle.
	drive_to_point(12.0, 0.0);

	run_dr4b(127, 800);
	run_dr4b(-127, 800);

	claw.set_value(true);
	pros::delay(400);
	claw.set_value(false);
}
