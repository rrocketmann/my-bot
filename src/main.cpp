#include "main.h"
#include "auton.hpp"

// Negate a port number to reverse that motor.
// Drive: 2 motors per side. DR4B: opposite-side motors, one reversed.
pros::MotorGroup left_drive({1, 2});
pros::MotorGroup right_drive({-9, -10});
pros::MotorGroup dr4b({11, -12});
pros::adi::DigitalOut claw('A');
pros::Controller master(pros::E_CONTROLLER_MASTER);

constexpr int kDriveDeadzone = 10;
constexpr int kLiftSpeed = 127;

int apply_deadzone(int value) {
	return (abs(value) < kDriveDeadzone) ? 0 : value;
}

void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Over Under");

	left_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	right_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	dr4b.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
	claw.set_value(false);
	position_init();
}

void disabled() {}

void competition_initialize() {}

void opcontrol() {
	left_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	right_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

	bool claw_closed = false;

	while (true) {
		position_update();

		// Arcade: left stick Y = forward, right stick X = turn
		int dir = apply_deadzone(master.get_analog(ANALOG_LEFT_Y));
		int turn = apply_deadzone(master.get_analog(ANALOG_RIGHT_X));
		left_drive.move(dir - turn);
		right_drive.move(dir + turn);

		// DR4B: R1 up, R2 down, hold brake when released
		if (master.get_digital(DIGITAL_R1)) {
			dr4b.move(kLiftSpeed);
		} else if (master.get_digital(DIGITAL_R2)) {
			dr4b.move(-kLiftSpeed);
		} else {
			dr4b.move(0);
		}

		// Pneumatic claw: L1 toggles open/close
		if (master.get_digital_new_press(DIGITAL_L1)) {
			claw_closed = !claw_closed;
			claw.set_value(claw_closed);
		}

		pros::delay(20);
	}
}
