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

// Voltage commands in the -127..127 joystick scale. The port signs
// carry the reversals: right drive and the left lift motor.
void command_drive(int left, int right);
void command_lift(int voltage);

// Brake modes are the PROS values: 0 coast, 1 brake, 2 hold.
void command_brake_drive(int mode);
void command_brake_lift(int mode);

#endif
