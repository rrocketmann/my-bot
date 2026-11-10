#include "main.h"
#include "auton.hpp"

#include <cstdint>
#include <cstdio>

// PROS motor_move returns ENODEV on an MC55. VEXcode motor55::spin calls
// vexMotorVoltageSet. Type 0 is an 11 W motor and type 1 is 5.5 W (12 V).
// An MC55 (type 2) is capped at 8 V. A voltage write with the current limit
// still at 0 produces no torque, so the limit goes out before the voltage.
// The value the controller applies on the wire is the open-loop PWM, -100..100.
extern "C" {
std::int32_t vexMotorVoltageSet(std::uint32_t index, std::int32_t voltage);
std::int32_t vexMotorVoltageLimitSet(std::uint32_t index, std::int32_t limit);
std::int32_t vexMotorPwmSet(std::uint32_t index, std::int32_t pwm);
std::int32_t vexMotorCurrentLimitSet(std::uint32_t index, std::int32_t limit);
std::int32_t vexMotorBrakeModeSet(std::uint32_t index, std::int32_t mode);
std::int32_t vexMotorTypeGet(std::uint32_t index);
}

void port_move(int port, int command) {
	const bool reversed = port < 0;
	if (reversed) {
		port = -port;
	}
	if (port < 1 || port > 21) {
		return;
	}
	const std::uint32_t index = static_cast<std::uint32_t>(port - 1);
	const int kind = vexMotorTypeGet(index);
	const int full_scale = (kind == 0 || kind == 1) ? 12000 : 8000;
	int millivolts = command * full_scale / 127;
	int pwm = command * 100 / 127;
	if (reversed) {
		millivolts = -millivolts;
		pwm = -pwm;
	}
	// Milliamps. MC55 hardware tops out near 1.2 A; smart motors use 2.5 A.
	const int current_limit = (kind == 0 || kind == 1) ? 2500 : 1200;
	vexMotorCurrentLimitSet(index, current_limit);
	vexMotorVoltageLimitSet(index, full_scale);
	vexMotorVoltageSet(index, millivolts);
	vexMotorPwmSet(index, pwm);
}

void command_drive(int left, int right) {
	port_move(3, left);
	port_move(14, left);
	port_move(-13, right);
	port_move(-12, right);
}

void command_lift(int voltage) {
	port_move(1, voltage);
	port_move(-2, voltage);
}

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

enum class Teleop { Override, KBot };

// Brain touch picks the scheme. The left third selects Override, the right
// third selects k-bot, and the middle confirms. Waiting out the window
// keeps Override. This is not called; opcontrol runs k-bot directly.
Teleop choose_teleop() {
	Teleop scheme = Teleop::Override;
	bool confirmed = false;
	std::int32_t seen_presses = pros::screen::touch_status().press_count;
	for (int elapsed = 0; elapsed < 3000 && !confirmed; elapsed += 20) {
		const char* name = scheme == Teleop::KBot ? "k-bot" : "Override";
		pros::screen::print(TEXT_MEDIUM, 0, "teleop: %s", name);
		pros::screen::print(TEXT_MEDIUM, 1, "left / right, center ok");
		const pros::screen_touch_status_s_t touch = pros::screen::touch_status();
		if (touch.press_count != seen_presses) {
			seen_presses = touch.press_count;
			if (touch.x < 160) {
				scheme = Teleop::Override;
			} else if (touch.x > 320) {
				scheme = Teleop::KBot;
			} else {
				confirmed = true;
			}
		}
		pros::delay(20);
	}
	return scheme;
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
	// VEXos drops every motor packet while this bit is set. A damaged
	// controller competition port sets the bit and the brain draws no
	// disabled icon. The controller screen is the visible signal.
	std::uint32_t cycle = pros::millis();
	while (pros::competition::is_disabled()) {
		pros::screen::print(TEXT_MEDIUM, 0, "DISABLED  %d mV", pros::battery::get_voltage());
		master.print(0, 0, "DISABLED");
		pros::Task::delay_until(&cycle, 500);
	}
}

void competition_initialize() {}

// Coast, then all four drive motors forward together. A per-port twitch is
// easy to miss, and a latched hold brake will eat a voltage command.
void test_motors() {
	const int drive_ports[] = {3, 14, 13, 12};
	for (int port : drive_ports) {
		vexMotorBrakeModeSet(static_cast<std::uint32_t>(port - 1), 0);
	}
	pros::screen::print(TEXT_MEDIUM, 0, "FORWARD  %d mV", pros::battery::get_voltage());
	master.print(0, 0, "FORWARD");
	command_drive(127, 127);
	pros::delay(1000);
	command_drive(0, 0);
	pros::screen::print(TEXT_MEDIUM, 0, "pulse done");
}

void teleop_override() {
	test_motors();

	bool claw_closed = false;
	std::uint32_t cycle = pros::millis();

	while (true) {
		position_update();

		// Tank: each stick drives its own side.
		int left = apply_deadzone(master.get_analog(ANALOG_LEFT_Y));
		int right = apply_deadzone(master.get_analog(ANALOG_RIGHT_Y));
		command_drive(left, right);

		pros::screen::print(TEXT_MEDIUM, 0, "Override  %d mV", pros::battery::get_voltage());

		// L2 runs the autonomous routine, then returns to driver control.
		if (master.get_digital_new_press(DIGITAL_L2)) {
			autonomous();
			left_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
			right_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
		}

		// DR4B: R1 up, R2 down, hold brake when released
		if (master.get_digital(DIGITAL_R1)) {
			command_lift(kLiftSpeed);
		} else if (master.get_digital(DIGITAL_R2)) {
			command_lift(-kLiftSpeed);
		} else {
			command_lift(0);
		}

		// Pneumatic claw: L1 toggles open/close
		if (master.get_digital_new_press(DIGITAL_L1)) {
			claw_closed = !claw_closed;
			claw.set_value(claw_closed);
		}

		pros::Task::delay_until(&cycle, 10);
	}
}

// A holds forward, B holds backward, left stick X turns (stick right
// speeds the left side). The two stick Y axes are tank drive when the
// right stick leaves the deadzone. X closes the claw, Y opens it.
// L1 raises the DR4B, L2 lowers it.
void teleop_kbot() {
	std::uint32_t cycle = pros::millis();
	while (true) {
		position_update();

		const int turn = apply_deadzone(master.get_analog(ANALOG_LEFT_X));
		const int left_stick = apply_deadzone(master.get_analog(ANALOG_LEFT_Y));
		const int right_stick = apply_deadzone(master.get_analog(ANALOG_RIGHT_Y));
		const bool forward = master.get_digital(DIGITAL_A);
		const bool backward = master.get_digital(DIGITAL_B);
		int left = 0;
		int right = 0;
		if (right_stick != 0) {
			left = left_stick;
			right = right_stick;
		} else {
			int dir = left_stick;
			if (forward && !backward) {
				dir = 127;
			} else if (backward && !forward) {
				dir = -127;
			}
			left = clamp_command(dir + turn);
			right = clamp_command(dir - turn);
		}
		command_drive(left, right);

		pros::screen::print(TEXT_MEDIUM, 0, "batt %d  ctrl %d  en %d", pros::battery::get_voltage(),
		                    master.is_connected() == 1 ? 1 : 0,
		                    pros::competition::is_disabled() ? 0 : 1);
		pros::screen::print(TEXT_MEDIUM, 1, "L%d R%d A%d B%d", left, right, forward ? 1 : 0,
		                    backward ? 1 : 0);

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

void opcontrol() {
	left_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	right_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	// One second of full forward, then the sticks and A/B take over.
	test_motors();
	teleop_kbot();
}
