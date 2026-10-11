// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.opmodes;

import static org.wpilib.units.Units.Seconds;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.limelightvision.FiducialTarget;
import com.limelightvision.Limelight;
import com.limelightvision.LimelightResults;
import frc.robot.Robot;
import frc.robot.subsystems.DriveMechanism;
import java.util.Arrays;
import java.util.Optional;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Utility;

/**
 * Measures each camera's {@link frc.robot.subsystems.vision.CameraCorrections} entry.
 *
 * <ol>
 *   <li>Put the robot <b>exactly</b> on the scoring spot of a real field element (use a jig).
 *   <li>Enable this. The robot backs straight away 0.6 m, slowly, while the cameras watch the tag.
 *   <li>Paste the printed lines into {@code CameraCorrections}.
 * </ol>
 *
 * <p>The tag is out of view at the scoring spot itself, which is why the robot backs away.
 */
@Utility(name = "Calibrate Cameras")
public class CalibrateCamerasOpMode extends PeriodicOpMode {
  // TODO: the tag on the element you park against.
  private static final int TAG_ID = 1;
  // The scoring spot in the tag's frame, from CAD - the same tagToGoal DriveToTag uses.
  private static final Transform2d TAG_TO_PARKED =
      new Transform2d(0.6, 0.0, Rotation2d.fromDegrees(180));
  private static final double BACK_OFF_SPEED = 0.15; // m/s
  private static final double BACK_OFF_SECONDS = 4.0;
  private static final double MAX_FRAME_AGE_SECONDS = 0.25;

  private final DriveMechanism drivetrain;
  private final Limelight[] cameras;
  private final Command backOff;
  private final long[] lastFrameIndex;
  // Per camera: sum of x, y, sin(yaw), cos(yaw), and the frame count.
  private final double[][] sums;

  private Pose2d expectedTag; // where the tag must be (odometry frame) if we parked exactly
  private double startTime;
  private boolean printed;

  public CalibrateCamerasOpMode(Robot robot) {
    drivetrain = robot.drivetrain;
    cameras = new Limelight[] {robot.limelightBR, robot.limelightBL};
    lastFrameIndex = new long[cameras.length];
    sums = new double[cameras.length][5];

    // Robot-relative, straight away from the tag.
    Translation2d tagInRobot = TAG_TO_PARKED.inverse().getTranslation();
    Translation2d away = tagInRobot.div(-tagInRobot.getNorm()).times(BACK_OFF_SPEED);
    SwerveRequest.ApplyRobotVelocity request =
        new SwerveRequest.ApplyRobotVelocity()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withVelocity(new ChassisVelocities(away.getX(), away.getY(), 0.0));
    backOff = drivetrain.applyRequest(() -> request).withTimeout(Seconds.of(BACK_OFF_SECONDS));
  }

  @Override
  public void start() {
    expectedTag = drivetrain.getOdometryPose().transformBy(TAG_TO_PARKED.inverse());
    for (double[] sum : sums) {
      Arrays.fill(sum, 0.0);
    }
    Arrays.fill(lastFrameIndex, -1);
    startTime = Utils.getCurrentTimeSeconds();
    printed = false;
    Scheduler.getDefault().schedule(backOff);
  }

  @Override
  public void periodic() {
    for (int i = 0; i < cameras.length; i++) {
      addFrame(i);
    }
    if (!printed && Utils.getCurrentTimeSeconds() - startTime > BACK_OFF_SECONDS + 0.5) {
      printed = true;
      printCorrections();
    }
  }

  @Override
  public void end() {
    Scheduler.getDefault().cancel(backOff);
  }

  private void addFrame(int i) {
    LimelightResults frame = cameras[i].getLatestResults();
    if (frame.frameIndex == lastFrameIndex[i]) {
      return;
    }
    lastFrameIndex[i] = frame.frameIndex;
    FiducialTarget tag = null;
    for (FiducialTarget target : frame.fiducialTargets) {
      if (target.fiducialId == TAG_ID) {
        tag = target;
      }
    }
    if (tag == null) {
      return;
    }
    Pose2d measured = tag.getTargetPose_RobotSpace().toPose2d();
    double frameAge =
        frame.getAgeSeconds()
            + (frame.captureLatencyMillis + frame.targetingLatencyMillis) / 1000.0;
    Optional<Pose2d> robotAtCapture =
        drivetrain.getOdometryPoseAt(Utils.getCurrentTimeSeconds() - frameAge);
    if (measured.getTranslation().getNorm() < 0.01
        || frameAge > MAX_FRAME_AGE_SECONDS
        || robotAtCapture.isEmpty()) {
      return;
    }
    // correction + measured = expected, so correction = expected - measured.
    Pose2d expected = expectedTag.relativeTo(robotAtCapture.get());
    Pose2d correction =
        expected.transformBy(
            new Transform2d(measured.getTranslation(), measured.getRotation()).inverse());
    sums[i][0] += correction.getX();
    sums[i][1] += correction.getY();
    sums[i][2] += correction.getRotation().getSin();
    sums[i][3] += correction.getRotation().getCos();
    sums[i][4] += 1.0;
  }

  private void printCorrections() {
    System.out.println("Paste into CameraCorrections:");
    for (int i = 0; i < cameras.length; i++) {
      double[] sum = sums[i];
      if (sum[4] < 20) {
        System.out.printf(
            "  %s saw the tag only %d times - check TAG_ID and the camera%n",
            cameras[i].getName(), (int) sum[4]);
        continue;
      }
      System.out.printf(
          "          \"%s\", new Transform2d(%.4f, %.4f, Rotation2d.fromDegrees(%.2f)),%n",
          cameras[i].getName(),
          sum[0] / sum[4],
          sum[1] / sum[4],
          Math.toDegrees(Math.atan2(sum[2], sum[3])));
    }
  }
}
