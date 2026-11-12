#include "main.h"
#include "auton.hpp"

#include <cstdint>

// 11 W V5 smart motors. Negate a port number to reverse that motor.
// Port 11 is the radio. Right drive is reversed so positive is forward.
// Left DR4B is reversed so both lift motors raise together.
//
// MotorGroup::move sends voltage only when PROS classifies the port as
// device type 2. A reading of 255 means PROS has no name for the device.
// These 11 W motors are still commanded through VEXos at 12 V and 2500 mA.
extern "C" {
std::int32_t vexMotorVoltageSet(std::uint32_t index, std::int32_t voltage);
std::int32_t vexMotorVoltageLimitSet(std::uint32_t index, std::int32_t limit);
std::int32_t vexMotorCurrentLimitSet(std::uint32_t index, std::int32_t limit);
std::int32_t vexMotorBrakeModeSet(std::uint32_t index, std::int32_t mode);
double vexMotorPositionGet(std::uint32_t index);
void vexMotorEncoderUnitsSet(std::uint32_t index, std::int32_t units);
}

// Degrees. Matches pros::E_MOTOR_ENCODER_DEGREES.
constexpr std::int32_t kEncoderDegrees = 0;
constexpr int kFullScaleMillivolts = 12000;
constexpr int kCurrentLimitMilliamps = 2500;

bool port_index(int port, std::uint32_t* index, bool* reversed) {
	*reversed = port < 0;
	if (*reversed) {
		port = -port;
	}
	if (port < 1 || port > 21) {
		return false;
	}
	*index = static_cast<std::uint32_t>(port - 1);
	return true;
}

pros::MotorGroup left_drive({3, 14});
pros::MotorGroup right_drive({-13, -12});
pros::MotorGroup dr4b({1, -2});
pros::adi::DigitalOut claw('B');
pros::Controller master(pros::E_CONTROLLER_MASTER);

void port_move(int port, int command) {
	bool reversed = false;
	std::uint32_t index = 0;
	if (!port_index(port, &index, &reversed)) {
		return;
	}
	int millivolts = command * kFullScaleMillivolts / 127;
	if (reversed) {
		millivolts = -millivolts;
	}
	vexMotorCurrentLimitSet(index, kCurrentLimitMilliamps);
	vexMotorVoltageLimitSet(index, kFullScaleMillivolts);
	vexMotorVoltageSet(index, millivolts);
}

void port_brake(int port, int mode) {
	bool reversed = false;
	std::uint32_t index = 0;
	if (!port_index(port, &index, &reversed)) {
		return;
	}
	vexMotorBrakeModeSet(index, mode);
}

double port_position_degrees(int port) {
	bool reversed = false;
	std::uint32_t index = 0;
	if (!port_index(port, &index, &reversed)) {
		return 0.0;
	}
	double degrees = vexMotorPositionGet(index);
	if (reversed) {
		degrees = -degrees;
	}
	return degrees;
}

void port_set_encoder_degrees(int port) {
	bool reversed = false;
	std::uint32_t index = 0;
	if (!port_index(port, &index, &reversed)) {
		return;
	}
	vexMotorEncoderUnitsSet(index, kEncoderDegrees);
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
	pros::screen::print(TEXT_MEDIUM, 1, "k-bot");

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
