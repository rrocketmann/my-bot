#ifndef AUTON_HPP_
#define AUTON_HPP_

// Field pose from the drive-motor encoders. x and y are inches.
// theta is radians, 0 along +x, counterclockwise positive.
struct Pose {
	double x;
	double y;
	double theta;
};

// Sample the motor encoders once so the first update has a zero delta.
void position_init();

// Integrate left and right motor travel into the field pose.
void position_update();

Pose position_get();
void position_set(Pose pose);

// Blocking move. Runs motor-encoder odometry and a distance/heading PID
// until the pose is within the settle distance or the timeout elapses.
void drive_to_point(double x, double y, int timeout_ms = 4000);

// Signed smart ports. A negative port reverses the command and the encoder.
// 1 DR4B right, 2 DR4B left, 3 back left, 14 front left,
// 13 back right, 12 front right.
constexpr int kLeftDrivePorts[] = {3, 14};
constexpr int kRightDrivePorts[] = {-13, -12};
constexpr int kLiftPorts[] = {1, -2};

// Shaft angle in degrees from the motor. A negative port negates it.
double port_position_degrees(int port);
// Ask the motor to report that angle in degrees.
void port_set_encoder_degrees(int port);

// Voltage commands in the -127..127 joystick scale. The port signs
// carry the reversals: right drive and the left lift motor.
void command_drive(int left, int right);
void command_lift(int voltage);

// Brake modes are the PROS values: 0 coast, 1 brake, 2 hold.
void command_brake_drive(int mode);
void command_brake_lift(int mode);

#endif
