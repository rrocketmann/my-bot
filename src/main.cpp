#include "main.h"
#include "auton.hpp"

#include <cstdint>
#include <cstdio>

// These take a zero-based smart-port index. motor55::spin in the VEXcode
// SDK does the same lookup, then vexDeviceMotorVoltageSet. PROS motor_move
// never gets that far: an MC55 is device type 255, and the motor claim
// returns ENODEV. Type 0 and 1 are 11 W and 5.5 W. Anything else, including
// an MC55 (motor type 2), is capped at 8 V.
extern "C" {
void* vexDeviceGetByIndex(std::uint32_t index);
std::int32_t vexMotorVoltageSet(std::uint32_t index, std::int32_t voltage);
std::int32_t vexMotorVoltageLimitSet(std::uint32_t index, std::int32_t limit);
std::int32_t vexMotorVoltageGet(std::uint32_t index);
std::int32_t vexMotorTypeGet(std::uint32_t index);
}

void port_move(int port, int command) {
	const bool reversed = port < 0;
	if (reversed) {
		port = -port;
	}
	const std::uint32_t index = static_cast<std::uint32_t>(port - 1);
	const int kind = vexMotorTypeGet(index);
	const int full_scale = (kind == 0 || kind == 1) ? 12000 : 8000;
	int millivolts = command * full_scale / 127;
	if (reversed) {
		millivolts = -millivolts;
	}
	vexMotorVoltageLimitSet(index, full_scale);
	vexMotorVoltageSet(index, millivolts);
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
	// Stay here until the brain is enabled. Motor commands are ignored
	// for the whole time this runs.
	std::uint32_t cycle = pros::millis();
	while (pros::competition::is_disabled()) {
		pros::screen::print(TEXT_MEDIUM, 0, "DISABLED  %d mV", pros::battery::get_voltage());
		pros::Task::delay_until(&cycle, 100);
	}
}

void competition_initialize() {}

// Spin one motor at a time so a dead port is obvious. Positive voltage is
// the motor's own forward direction, ignoring the drive reversal flags.
void test_motors() {
	struct MotorCheck {
		int port;
		const char* name;
	};
	const MotorCheck motors[] = {
		{1, "DR4B R"},  {2, "DR4B L"},  {3, "back L"},
		{14, "front L"}, {13, "back R"}, {12, "front R"},
	};

	pros::screen::print(TEXT_MEDIUM, 0, "motor test  %d mV", pros::battery::get_voltage());
	std::printf("batt %d disabled %d\n", pros::battery::get_voltage(),
	            pros::competition::is_disabled());
	std::fflush(stdout);
	pros::delay(1500);

	for (const MotorCheck& motor : motors) {
		const std::uint32_t index = static_cast<std::uint32_t>(motor.port - 1);
		void* device = vexDeviceGetByIndex(index);
		const int plugged = static_cast<int>(pros::Device::get_plugged_type(motor.port));
		const int motor_type = vexMotorTypeGet(index);
		const int battery = pros::battery::get_voltage();
		const int enabled = pros::competition::is_disabled() ? 0 : 1;
		pros::screen::print(TEXT_MEDIUM, 0, "batt %d mV  en %d", battery, enabled);
		pros::screen::print(TEXT_MEDIUM, 1, "port %d %s", motor.port, motor.name);
		pros::screen::print(TEXT_MEDIUM, 2, "plug %d type %d dev %d", plugged, motor_type,
		                    device != nullptr);
		std::printf("start port %d %s batt %d en %d plug %d type %d dev %d\n", motor.port,
		            motor.name, battery, enabled, plugged, motor_type, device != nullptr);
		std::fflush(stdout);

		// 6 V is inside the 8 V MC55 cap and enough to see a smart motor turn.
		vexMotorVoltageLimitSet(index, 8000);
		vexMotorVoltageSet(index, 6000);
		pros::delay(700);
		const int readback = vexMotorVoltageGet(index);
		pros::screen::print(TEXT_MEDIUM, 3, "read %d mV", readback);
		std::printf("read port %d %d mV\n", motor.port, readback);
		std::fflush(stdout);
		vexMotorVoltageSet(index, 0);

		std::printf("done port %d %s\n", motor.port, motor.name);
		std::fflush(stdout);
		pros::delay(800);
	}

	pros::screen::print(TEXT_MEDIUM, 0, "motor test done");
	std::printf("motor test done\n");
	std::fflush(stdout);
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

// Tank buttons plus a single stick. A holds forward, B holds backward.
// Left stick X turns: stick right speeds the left side. X closes the claw,
// Y opens it. L1 raises the DR4B, L2 lowers it.
void teleop_kbot() {
	std::uint32_t cycle = pros::millis();
	while (true) {
		position_update();

		int turn = apply_deadzone(master.get_analog(ANALOG_LEFT_X));
		const bool forward = master.get_digital(DIGITAL_A);
		const bool backward = master.get_digital(DIGITAL_B);
		int dir = 0;
		if (forward && !backward) {
			dir = 127;
		} else if (backward && !forward) {
			dir = -127;
		}
		command_drive(clamp_command(dir + turn), clamp_command(dir - turn));

		pros::screen::print(TEXT_MEDIUM, 0, "batt %d  ctrl %d  en %d", pros::battery::get_voltage(),
		                    master.is_connected() == 1 ? 1 : 0,
		                    pros::competition::is_disabled() ? 0 : 1);
		pros::screen::print(TEXT_MEDIUM, 1, "A%d B%d X%d L%d turn %d", forward ? 1 : 0,
		                    backward ? 1 : 0, master.get_digital(DIGITAL_X) ? 1 : 0,
		                    master.get_digital(DIGITAL_L1) ? 1 : 0, turn);

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
	// Pulses every motor with no controller input, and prints battery,
	// enable state, and the device the firmware reports on that port.
	test_motors();
	teleop_kbot();
}
