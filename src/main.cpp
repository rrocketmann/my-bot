#include "main.h"
#include "auton.hpp"

#include <cstdint>

// Negate a port number to reverse that motor.
// 1 DR4B right, 2 DR4B left, 3 back left, 14 front left,
// 13 back right, 12 front right. Port 11 is the radio.
// Right drive is reversed so positive is forward. Left DR4B is
// reversed so both lift motors raise together.
//
// MotorGroup::move only accepts smart-port device type 2. An MC55
// enumerates as type 255, so that call returns an error and the motor
// stays still. VEXcode motor55::spin calls the VEXos functions below.
// vexMotorTypeGet: 0 is 11 W, 1 is 5.5 W (both 12 V), 2 is an MC55 (8 V).
// A voltage write with the current limit still at 0 produces no torque.
// The value an MC55 applies on the wire is the open-loop PWM.
extern "C" {
std::int32_t vexMotorVoltageSet(std::uint32_t index, std::int32_t voltage);
std::int32_t vexMotorVoltageLimitSet(std::uint32_t index, std::int32_t limit);
std::int32_t vexMotorPwmSet(std::uint32_t index, std::int32_t pwm);
std::int32_t vexMotorCurrentLimitSet(std::uint32_t index, std::int32_t limit);
std::int32_t vexMotorBrakeModeSet(std::uint32_t index, std::int32_t mode);
std::int32_t vexMotorTypeGet(std::uint32_t index);
}

pros::MotorGroup left_drive({3, 14});
pros::MotorGroup right_drive({-13, -12});
pros::MotorGroup dr4b({1, -2});
pros::adi::DigitalOut claw('B');
pros::Controller master(pros::E_CONTROLLER_MASTER);

constexpr int kLeftDrivePorts[] = {3, 14};
constexpr int kRightDrivePorts[] = {-13, -12};
constexpr int kLiftPorts[] = {1, -2};

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
	// Milliamps. An MC55 tops out near 1.2 A; a smart motor uses 2.5 A.
	const int current_limit = (kind == 0 || kind == 1) ? 2500 : 1200;
	vexMotorCurrentLimitSet(index, current_limit);
	vexMotorVoltageLimitSet(index, full_scale);
	vexMotorVoltageSet(index, millivolts);
	vexMotorPwmSet(index, pwm);
}

void port_brake(int port, int mode) {
	if (port < 0) {
		port = -port;
	}
	if (port < 1 || port > 21) {
		return;
	}
	vexMotorBrakeModeSet(static_cast<std::uint32_t>(port - 1), mode);
}

void command_ports(const int* ports, int count, int command) {
	for (int i = 0; i < count; ++i) {
		port_move(ports[i], command);
	}
}

void brake_ports(const int* ports, int count, int mode) {
	for (int i = 0; i < count; ++i) {
		port_brake(ports[i], mode);
	}
}

void command_drive(int left, int right) {
	command_ports(kLeftDrivePorts, 2, left);
	command_ports(kRightDrivePorts, 2, right);
}

void command_lift(int voltage) {
	command_ports(kLiftPorts, 2, voltage);
}

void command_brake_drive(int mode) {
	left_drive.set_brake_mode_all(static_cast<pros::motor_brake_mode_e_t>(mode));
	right_drive.set_brake_mode_all(static_cast<pros::motor_brake_mode_e_t>(mode));
	brake_ports(kLeftDrivePorts, 2, mode);
	brake_ports(kRightDrivePorts, 2, mode);
}

void command_brake_lift(int mode) {
	dr4b.set_brake_mode_all(static_cast<pros::motor_brake_mode_e_t>(mode));
	brake_ports(kLiftPorts, 2, mode);
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
	pros::screen::print(TEXT_MEDIUM, 1, "k-bot init");

	int left_pos = left_drive.get_position_all()[0];
	int right_pos = right_drive.get_position_all()[0];
	pros::screen::print(TEXT_MEDIUM, 2, "init pos L:%d R:%d", left_pos, right_pos);

	command_brake_drive(pros::E_MOTOR_BRAKE_COAST);
	command_brake_lift(pros::E_MOTOR_BRAKE_HOLD);
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
	command_brake_drive(pros::E_MOTOR_BRAKE_COAST);

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
