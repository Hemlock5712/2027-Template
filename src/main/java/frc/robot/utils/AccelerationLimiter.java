// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import frc.robot.generated.TunerConstants;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.util.Units;

/**
 * How fast the robot's velocity may change, from physics. Give it the velocity you have and the one
 * you want; it returns the closest one the robot can reach this loop without slipping:
 *
 * <ol>
 *   <li><b>Turning:</b> the change in spin is capped (sudden heading changes slip the wheels).
 *   <li><b>Motors:</b> when speeding up, no more than the drive motors can still add at this speed.
 *   <li><b>Traction:</b> driving and turning together, no more than the tires can grip.
 * </ol>
 *
 * <p>kSlipCurrent in TunerConstants is only the hardware ceiling; these numbers decide how hard we
 * actually push. Tuned in tools/alignsim (docs/alignment/studies/controllers.md).
 */
public final class AccelerationLimiter {
  // Keep TRACTION_ACCEL above DriveToPose's DECEL, and ANGULAR_ACCEL above its ANGULAR_DECEL, or
  // the robot can't brake as planned and overshoots.
  private static final double TRACTION_ACCEL = 10.0; // m/s^2 (~1 g), shared by driving and turning
  private static final double ANGULAR_ACCEL = 8.0; // rad/s^2
  private static final double MODEL_CURRENT = 150.0; // A per drive motor, for the torque limit
  private static final double ROBOT_MASS = 60.0; // kg with battery and bumpers. TODO: weigh it
  // Kraken X60 FOC, CTRE dyno data: 9.362 Nm and 476.1 A at stall, 5785 rpm free. Free current
  // stays 0 so torque hits zero at the real free speed (DCMotor counts friction as push).
  private static final DCMotor DRIVE_MOTOR =
      new DCMotor(12.0, 9.362, 476.1, 0.0, Units.rotationsPerMinuteToRadiansPerSecond(5785), 1);
  private static final double GEAR_RATIO = TunerConstants.FrontLeft.DriveMotorGearRatio;
  private static final double WHEEL_RADIUS = TunerConstants.FrontLeft.WheelRadius; // m
  private static final double BASE_RADIUS = // m, robot center to a wheel
      Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY);

  private AccelerationLimiter() {}

  /**
   * The velocity to command this loop: as close to {@code wanted} as physics allows, starting from
   * {@code current}. Both field-relative; {@code dt} in seconds.
   */
  public static ChassisVelocities limit(
      ChassisVelocities current, ChassisVelocities wanted, double dt) {
    double ax = (wanted.vx - current.vx) / dt;
    double ay = (wanted.vy - current.vy) / dt;
    double alpha =
        Math.max(-ANGULAR_ACCEL, Math.min(ANGULAR_ACCEL, (wanted.omega - current.omega) / dt));

    // Motors: only speeding up is limited; less torque is left as the wheels near free speed.
    // Spin-up isn't counted: using the worst-case wheel speed, it slowed turns while driving (Q7).
    if (ax * current.vx + ay * current.vy > 0.0) {
      double wheelSpeed =
          Math.hypot(current.vx, current.vy) + Math.abs(current.omega) * BASE_RADIUS;
      double motorLimit = motorAccel(wheelSpeed);
      double a = Math.hypot(ax, ay);
      if (a > motorLimit) {
        ax *= motorLimit / a;
        ay *= motorLimit / a;
      }
    }

    // Traction: driving and turning share the tread's grip. Scaled as one vector, so paths stay
    // straight.
    double total = Math.hypot(Math.hypot(ax, ay), alpha * BASE_RADIUS);
    if (total > TRACTION_ACCEL) {
      double scale = TRACTION_ACCEL / total;
      ax *= scale;
      ay *= scale;
      alpha *= scale;
    }
    return new ChassisVelocities(
        current.vx + ax * dt, current.vy + ay * dt, current.omega + alpha * dt);
  }

  /** Most acceleration the four drive motors can still add at this wheel speed (m/s^2). */
  public static double motorAccel(double wheelSpeed) {
    double motorSpeed = wheelSpeed / WHEEL_RADIUS * GEAR_RATIO; // rad/s
    double amps = Math.min(MODEL_CURRENT, DRIVE_MOTOR.getCurrent(motorSpeed, 12.0));
    double wheelForce = DRIVE_MOTOR.getTorque(Math.max(0.0, amps)) * GEAR_RATIO / WHEEL_RADIUS;
    return 4.0 * wheelForce / ROBOT_MASS;
  }
}
