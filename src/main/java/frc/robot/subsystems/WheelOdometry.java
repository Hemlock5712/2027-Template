// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import java.util.Optional;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.interpolation.TimeInterpolatableBuffer;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveDriveOdometry;

/**
 * The robot's pose from wheels + gyro only. Vision never touches it, so it drifts slowly but never
 * jumps - this is the frame precise alignment works in (ROS calls it the "odom" frame).
 *
 * <p>Fed by the drivetrain's odometry thread (~250 Hz) and read by the main loop, so every method
 * is {@code synchronized}.
 */
public class WheelOdometry {
  private final SwerveDriveKinematics kinematics;
  private final TimeInterpolatableBuffer<Pose2d> history =
      TimeInterpolatableBuffer.createBuffer(1.5); // seconds kept

  private SwerveDriveOdometry odometry; // created from the first sample
  private Pose2d latest = new Pose2d();

  public WheelOdometry(SwerveDriveKinematics kinematics) {
    this.kinematics = kinematics;
  }

  /** Adds one odometry sample. Called on the odometry thread - keep it fast. */
  public synchronized void update(SwerveDriveState state) {
    if (state.ModulePositions == null) {
      return;
    }
    if (odometry == null) {
      odometry = new SwerveDriveOdometry(kinematics, state.RawHeading, state.ModulePositions);
    }
    latest = odometry.update(state.RawHeading, state.ModulePositions);
    history.addSample(state.Timestamp, latest);
  }

  /** The newest wheel-only pose. */
  public synchronized Pose2d getPose() {
    return latest;
  }

  /**
   * The wheel-only pose at a past time, interpolated between samples.
   *
   * @param timestampSeconds time in the Phoenix timebase ({@code Utils.getCurrentTimeSeconds()})
   * @return empty until the first sample arrives
   */
  public synchronized Optional<Pose2d> getPoseAt(double timestampSeconds) {
    return history.getSample(timestampSeconds);
  }
}
