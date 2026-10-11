// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.DriveMechanism;
import frc.robot.utils.AllianceFlip;
import frc.robot.utils.ClassicCommand;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * Drive to a field pose by commanding velocity directly: each loop, head straight at the goal at
 * the fastest speed that can still stop in time, then limit how fast that velocity may change. The
 * goal may move every loop - {@link DriveToTag} and {@link DriveToPiece} build on that.
 *
 * <p>Classic-style Commands v3 command on {@link ClassicCommand}.
 */
public class DriveToPose extends ClassicCommand {
  // Translation limits. DECEL is what we plan to brake with; ACCEL is the hard limit on any
  // change in velocity, so keep DECEL below it. TODO: tune to the drivetrain's real capability.
  private static final double MAX_SPEED = 3.5; // m/s
  private static final double DECEL = 3.0; // m/s^2
  private static final double ACCEL = 6.0; // m/s^2
  // Close to the goal, speed = KP * distance instead of the braking curve (no chatter at the end).
  private static final double KP = 4.0; // (m/s) per m
  // Brake this much early to cover sensor + CAN delay.
  private static final double REACTION_TIME = 0.05; // s

  // Same three ideas for heading.
  private static final double MAX_OMEGA = 6.0; // rad/s
  private static final double ANGULAR_DECEL = 8.0; // rad/s^2
  private static final double ANGULAR_ACCEL = 20.0; // rad/s^2
  private static final double KP_HEADING = 5.0; // (rad/s) per rad

  // A pass-through waypoint (see withEndSpeed) counts as reached inside this radius.
  private static final double WAYPOINT_TOLERANCE = 0.15; // m
  // "Stopped" for isFinished().
  private static final double SETTLED_SPEED = 0.05; // m/s

  private final DriveMechanism drivetrain;
  private final Pose2d blueGoal; // null when a goal supplier is used instead
  private Supplier<Pose2d> goalSupplier;

  private double positionTolerance = 0.02; // m
  private double headingTolerance = Math.toRadians(2.0); // rad
  private double endSpeed = 0.0; // m/s; > 0 makes this a pass-through waypoint

  // Field-relative velocity request, blue-origin (the same frame as the pose). Closed-loop
  // velocity uses the drive gains in TunerConstants, so slow final moves still happen.
  private final SwerveRequest.ApplyFieldVelocity driveRequest =
      new SwerveRequest.ApplyFieldVelocity()
          .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance)
          .withDriveRequestType(DriveRequestType.Velocity);

  private ChassisVelocities commanded = new ChassisVelocities();
  private double lastTime;
  private double distance = Double.POSITIVE_INFINITY;
  private double headingError = Double.POSITIVE_INFINITY;

  /**
   * @param drivetrain the swerve drive to command
   * @param blueGoal the field pose to drive to, including heading. <b>Always write this for the
   *     blue alliance</b> - on red it is flipped for you, see {@link AllianceFlip}.
   */
  public DriveToPose(DriveMechanism drivetrain, Pose2d blueGoal) {
    this("DriveToPose", drivetrain, blueGoal, null);
  }

  /**
   * Drives to a goal that can change every loop. The supplier returns a field pose (blue-origin, as
   * odometry reports it) and is <b>not</b> flipped for red.
   */
  public DriveToPose(DriveMechanism drivetrain, Supplier<Pose2d> goalSupplier) {
    this("DriveToPose", drivetrain, null, goalSupplier);
  }

  /** For subclasses that override {@link #goal()}. */
  protected DriveToPose(String name, DriveMechanism drivetrain) {
    this(name, drivetrain, null, drivetrain::getPose);
  }

  private DriveToPose(
      String name, DriveMechanism drivetrain, Pose2d blueGoal, Supplier<Pose2d> goalSupplier) {
    super(name, drivetrain); // name + requirement, like v2 addRequirements(drivetrain)
    this.drivetrain = drivetrain;
    this.blueGoal = blueGoal;
    this.goalSupplier = goalSupplier;
  }

  /** Finish tolerances (defaults: 2 cm, 2 degrees). */
  public DriveToPose withTolerance(double meters, double degrees) {
    positionTolerance = meters;
    headingTolerance = Math.toRadians(degrees);
    return this;
  }

  /**
   * Makes this a pass-through waypoint: arrive at {@code metersPerSecond} instead of stopping, and
   * finish as soon as the robot is close (heading isn't checked). Chain the next leg right after.
   */
  public DriveToPose withEndSpeed(double metersPerSecond) {
    endSpeed = metersPerSecond;
    return this;
  }

  /** The goal for this loop. Override to supply a moving goal. */
  protected Pose2d goal() {
    return goalSupplier.get();
  }

  /** Picks the goal for our alliance and starts from the robot's current velocity. */
  @Override
  protected void initialize() {
    if (blueGoal != null) {
      // Resolved here, not in the constructor: the alliance isn't known when an OpMode is built.
      Pose2d activeGoal = AllianceFlip.apply(blueGoal);
      goalSupplier = () -> activeGoal;
    }
    commanded = drivetrain.getFieldVelocity();
    lastTime = Utils.getCurrentTimeSeconds();
    distance = Double.POSITIVE_INFINITY;
    headingError = Double.POSITIVE_INFINITY;
  }

  @Override
  protected void execute() {
    double now = Utils.getCurrentTimeSeconds();
    double dt = Math.min(Math.max(now - lastTime, 0.001), 0.1); // guards a skipped loop
    lastTime = now;

    Pose2d pose = drivetrain.getPose();
    Pose2d goal = goal();
    Translation2d toGoal = goal.getTranslation().minus(pose.getTranslation());
    distance = toGoal.getNorm();
    headingError = goal.getRotation().minus(pose.getRotation()).getRadians(); // -pi..pi

    // Speed toward the goal: the fastest speed that can still brake to endSpeed in the distance
    // left, capped by MAX_SPEED and (when stopping) by the gentle KP line near the goal.
    double speedNow = Math.hypot(commanded.vx, commanded.vy);
    double brakingDistance = Math.max(0.0, distance - speedNow * REACTION_TIME);
    double speed =
        Math.min(MAX_SPEED, Math.sqrt(endSpeed * endSpeed + 2.0 * DECEL * brakingDistance));
    if (endSpeed == 0.0) {
      speed = Math.min(speed, KP * distance);
    }
    double vx = distance > 1e-6 ? toGoal.getX() / distance * speed : 0.0;
    double vy = distance > 1e-6 ? toGoal.getY() / distance * speed : 0.0;

    double turnSpeed =
        Math.min(
            MAX_OMEGA,
            Math.min(
                Math.sqrt(2.0 * ANGULAR_DECEL * Math.abs(headingError)),
                KP_HEADING * Math.abs(headingError)));
    double omega = Math.copySign(turnSpeed, headingError);

    // Limit the change as one vector, so the direction of travel never bends while speeding up.
    double dvx = vx - commanded.vx;
    double dvy = vy - commanded.vy;
    double dv = Math.hypot(dvx, dvy);
    if (dv > ACCEL * dt) {
      dvx *= ACCEL * dt / dv;
      dvy *= ACCEL * dt / dv;
    }
    double dOmega =
        Math.max(-ANGULAR_ACCEL * dt, Math.min(ANGULAR_ACCEL * dt, omega - commanded.omega));
    commanded =
        new ChassisVelocities(commanded.vx + dvx, commanded.vy + dvy, commanded.omega + dOmega);

    drivetrain.setControl(driveRequest.withVelocity(commanded));

    Logger.recordOutput(name() + "/Goal", goal);
    Logger.recordOutput(name() + "/DistanceMeters", distance);
    Logger.recordOutput(name() + "/HeadingErrorDegrees", Math.toDegrees(headingError));
    Logger.recordOutput(name() + "/CommandedVelocity", commanded);
  }

  /** Waypoints finish when close; otherwise at the goal, facing the right way, and stopped. */
  @Override
  protected boolean isFinished() {
    if (endSpeed > 0.0) {
      return distance < WAYPOINT_TOLERANCE;
    }
    ChassisVelocities measured = drivetrain.getFieldVelocity();
    return distance < positionTolerance
        && Math.abs(headingError) < headingTolerance
        && Math.hypot(measured.vx, measured.vy) < SETTLED_SPEED;
  }

  /** Stops the drivetrain - except after a waypoint, where the next leg takes over at speed. */
  @Override
  protected void end(boolean interrupted) {
    if (interrupted || endSpeed == 0.0) {
      drivetrain.setControl(new SwerveRequest.Idle());
    }
  }
}
