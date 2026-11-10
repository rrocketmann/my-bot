#include "main.h"
#include "auton.hpp"

// 11 W V5 smart motors for the 2026-2027 Override season.
// Negate a port number to reverse that motor.
// 1 DR4B right, 2 DR4B left, 3 back left, 14 front left,
// 13 back right, 12 front right. Port 11 is the radio.
// Right drive is reversed so positive is forward. Left DR4B is
// reversed so both lift motors raise together.
pros::MotorGroup left_drive({3, 14});
pros::MotorGroup right_drive({-13, -12});
pros::MotorGroup dr4b({1, -2});
pros::adi::DigitalOut claw('B');
pros::Controller master(pros::E_CONTROLLER_MASTER);

void command_drive(int left, int right) {
	left_drive.move(left);
	right_drive.move(right);
}

void command_lift(int voltage) {
	dr4b.move(voltage);
}

constexpr int kDriveDeadzone = 10;
constexpr int kLiftSpeed = 127;

int apply_deadzone(int value) {
	return (abs(value) < kDriveDeadzone) ? 0 : value;
}

int clamp_command(int value) {
	if (value > 127) {
		return 127;
	}
	if (value < -127) {
		return -127;
	}
	return value;
}

void initialize() {
	pros::screen::set_pen(pros::Color::white);
	pros::screen::print(TEXT_MEDIUM, 1, "k-bot");

	left_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	right_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	dr4b.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
	claw.set_value(false);
	position_init();
}

void disabled() {
	// VEXos drops motor output while this bit is set.
	std::uint32_t cycle = pros::millis();
	while (pros::competition::is_disabled()) {
		pros::screen::print(TEXT_MEDIUM, 0, "DISABLED  %d mV", pros::battery::get_voltage());
		pros::Task::delay_until(&cycle, 100);
	}
}

void competition_initialize() {}

// A holds forward, B holds backward. Left stick X turns: stick right
// speeds the left side. X closes the claw, Y opens it. L1 raises the
// DR4B, L2 lowers it.
void opcontrol() {
	left_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	right_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

	std::uint32_t cycle = pros::millis();
	while (true) {
		position_update();

		const int turn = apply_deadzone(master.get_analog(ANALOG_LEFT_X));
		const bool forward = master.get_digital(DIGITAL_A);
		const bool backward = master.get_digital(DIGITAL_B);
		int dir = 0;
		if (forward && !backward) {
			dir = 127;
		} else if (backward && !forward) {
			dir = -127;
		}
		command_drive(clamp_command(dir + turn), clamp_command(dir - turn));

		if (master.get_digital(DIGITAL_L1)) {
			command_lift(kLiftSpeed);
		} else if (master.get_digital(DIGITAL_L2)) {
			command_lift(-kLiftSpeed);
		} else {
			command_lift(0);
		}

		if (master.get_digital(DIGITAL_X)) {
			claw.set_value(true);
		} else if (master.get_digital(DIGITAL_Y)) {
			claw.set_value(false);
		}

		pros::Task::delay_until(&cycle, 10);
	}
}
