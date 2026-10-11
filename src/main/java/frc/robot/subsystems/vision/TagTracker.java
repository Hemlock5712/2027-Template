// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import com.ctre.phoenix6.Utils;
import com.limelightvision.FiducialTarget;
import com.limelightvision.Limelight;
import com.limelightvision.LimelightResults;
import frc.robot.subsystems.DriveMechanism;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;

/**
 * Tracks where one AprilTag really is, as a pose in the wheel-odometry frame ({@link
 * DriveMechanism#getOdometryPose()}). This is the "local pose": it never uses the field map or the
 * vision-corrected field pose, so a misplaced field element or a vision jump can't pull alignment
 * off.
 *
 * <p>Each frame is placed using the odometry pose from the moment the picture was taken (latency
 * compensation), then averaged into one estimate. When the tag leaves view - too close, blocked -
 * odometry carries the estimate the rest of the way.
 *
 * <p>Call {@link #update()} every loop while you use it ({@link frc.robot.commands.DriveToTag}
 * does).
 */
public class TagTracker {
  // Frames outside these limits are skipped. TODO: tune from logs.
  private static final double MAX_TAG_DISTANCE_METERS = 3.5;
  private static final double MAX_AMBIGUITY = 0.3;
  private static final double MAX_FRAME_AGE_SECONDS = 0.25;
  // A frame this far from the estimate is a wrong tag or a flipped solve - unless 3 in a row agree.
  private static final double JUMP_METERS = 0.25;
  private static final int JUMPS_TO_RESTART = 3;
  private static final int FRAMES_TO_LOCK = 3;

  // Per-frame noise (1 sigma), growing with distance d: xy = floor + k * d^2, yaw = floor + k * d.
  private static final double XY_NOISE_FLOOR = 0.005; // m
  private static final double XY_NOISE_PER_M2 = 0.01; // m per m^2
  private static final double YAW_NOISE_FLOOR = Math.toRadians(0.5);
  private static final double YAW_NOISE_PER_M = Math.toRadians(1.5);
  // How far the tag may appear to move per frame (odometry drift); keeps the filter listening.
  private static final double XY_DRIFT = 0.002; // m
  private static final double YAW_DRIFT = Math.toRadians(0.1);

  private final DriveMechanism drivetrain;
  private final int tagId;
  private final Limelight[] cameras;
  private final long[] lastFrameIndex;
  private final String logPrefix;

  private Pose2d tagPose; // odometry frame; null until the first good frame
  private final double[] variance = new double[3]; // x, y, yaw of tagPose
  private int goodFrames;
  private int jumpsInARow;
  private double lastSeenTime;

  /**
   * @param tagId the AprilTag to track
   * @param cameras every camera that might see it; frames from all of them are combined
   */
  public TagTracker(DriveMechanism drivetrain, int tagId, Limelight... cameras) {
    this.drivetrain = drivetrain;
    this.tagId = tagId;
    this.cameras = cameras;
    this.lastFrameIndex = new long[cameras.length];
    this.logPrefix = "TagTracker/" + tagId + "/";
    reset();
  }

  /** Forgets the estimate. Call when starting a new approach. */
  public void reset() {
    tagPose = null;
    goodFrames = 0;
    jumpsInARow = 0;
    Arrays.fill(lastFrameIndex, -1);
  }

  /**
   * Reads the newest frame from each camera and folds any sighting of our tag into the estimate.
   */
  public void update() {
    List<Pose2d> tagInRobotLog = new ArrayList<>();
    List<Pose2d> robotAtCaptureLog = new ArrayList<>();
    List<Double> frameAgeLog = new ArrayList<>();

    for (int i = 0; i < cameras.length; i++) {
      LimelightResults frame = cameras[i].getLatestResults();
      if (frame.frameIndex == lastFrameIndex[i]) {
        continue; // nothing new from this camera
      }
      lastFrameIndex[i] = frame.frameIndex;

      FiducialTarget tag = findTag(frame);
      if (tag == null || tag.ambiguity > MAX_AMBIGUITY) {
        continue;
      }
      Pose3d tagInRobot3d = tag.getTargetPose_RobotSpace();
      double distance = tagInRobot3d.getTranslation().toTranslation2d().getNorm();
      if (distance < 0.01 || distance > MAX_TAG_DISTANCE_METERS) {
        continue; // ~0 means the camera sent no 3D solve for this tag
      }

      // How long ago the picture was taken: time since it arrived + the camera's own latency.
      double frameAge =
          frame.getAgeSeconds()
              + (frame.captureLatencyMillis + frame.targetingLatencyMillis) / 1000.0;
      if (frameAge > MAX_FRAME_AGE_SECONDS) {
        continue;
      }
      Optional<Pose2d> robotAtCapture =
          drivetrain.getOdometryPoseAt(Utils.getCurrentTimeSeconds() - frameAge);
      if (robotAtCapture.isEmpty()) {
        continue;
      }

      Pose2d tagInRobot = tagInRobot3d.toPose2d();
      Pose2d measured =
          robotAtCapture
              .get()
              .transformBy(new Transform2d(tagInRobot.getTranslation(), tagInRobot.getRotation()));
      fuse(measured, distance);

      tagInRobotLog.add(tagInRobot);
      robotAtCaptureLog.add(robotAtCapture.get());
      frameAgeLog.add(frameAge);
    }

    // Raw inputs too, so other filters/strategies can be replayed offline from the log.
    Logger.recordOutput(logPrefix + "TagInRobot", tagInRobotLog.toArray(Pose2d[]::new));
    Logger.recordOutput(logPrefix + "RobotAtCapture", robotAtCaptureLog.toArray(Pose2d[]::new));
    Logger.recordOutput(
        logPrefix + "FrameAgeSeconds",
        frameAgeLog.stream().mapToDouble(Double::doubleValue).toArray());
    Logger.recordOutput(logPrefix + "Locked", hasLock());
    Logger.recordOutput(logPrefix + "SecondsSinceSeen", getSecondsSinceSeen());
    getTagPose().ifPresent(pose -> Logger.recordOutput(logPrefix + "TagPose", pose));
  }

  /** True once a few frames agree on where the tag is. Stays true while odometry coasts. */
  public boolean hasLock() {
    return tagPose != null && goodFrames >= FRAMES_TO_LOCK;
  }

  /** The tag's pose in the wheel-odometry frame (+X out of the tag face, +Y to its left). */
  public Optional<Pose2d> getTagPose() {
    return Optional.ofNullable(tagPose);
  }

  /**
   * The spot {@code tagToGoal} away from the tag, as a field pose the drivetrain can drive to.
   *
   * <p>The goal is measured from where the robot is now (wheel odometry), then placed onto the
   * field pose. A vision jump in the field pose moves the robot and the goal together, so it
   * cancels.
   *
   * @return empty until {@link #hasLock()}
   */
  public Optional<Pose2d> getFieldGoal(Transform2d tagToGoal) {
    if (!hasLock()) {
      return Optional.empty();
    }
    Pose2d goalInOdometry = tagPose.transformBy(tagToGoal);
    Transform2d robotToGoal = goalInOdometry.minus(drivetrain.getOdometryPose());
    return Optional.of(drivetrain.getPose().transformBy(robotToGoal));
  }

  /** Where the robot is right now in the tag's frame - what to copy into a tagToGoal. */
  public Optional<Pose2d> getRobotInTag() {
    return getTagPose().map(tag -> drivetrain.getOdometryPose().relativeTo(tag));
  }

  /** Seconds since a frame last updated the estimate (infinite if never). */
  public double getSecondsSinceSeen() {
    return tagPose == null
        ? Double.POSITIVE_INFINITY
        : Utils.getCurrentTimeSeconds() - lastSeenTime;
  }

  private FiducialTarget findTag(LimelightResults frame) {
    for (FiducialTarget target : frame.fiducialTargets) {
      if (target.fiducialId == tagId) {
        return target;
      }
    }
    return null;
  }

  /** Folds one measurement into the estimate - a tiny Kalman filter per axis (x, y, yaw). */
  private void fuse(Pose2d measured, double distance) {
    double xyNoise = XY_NOISE_FLOOR + XY_NOISE_PER_M2 * distance * distance;
    double yawNoise = YAW_NOISE_FLOOR + YAW_NOISE_PER_M * distance;
    double[] noise = {xyNoise * xyNoise, xyNoise * xyNoise, yawNoise * yawNoise};

    if (tagPose == null) {
      restart(measured, noise);
      return;
    }
    double dx = measured.getX() - tagPose.getX();
    double dy = measured.getY() - tagPose.getY();
    double dYaw = measured.getRotation().minus(tagPose.getRotation()).getRadians();
    if (Math.hypot(dx, dy) > JUMP_METERS) {
      jumpsInARow++;
      if (jumpsInARow >= JUMPS_TO_RESTART) {
        restart(measured, noise); // the old estimate was the wrong one
      }
      return;
    }
    jumpsInARow = 0;

    double[] drift = {XY_DRIFT * XY_DRIFT, XY_DRIFT * XY_DRIFT, YAW_DRIFT * YAW_DRIFT};
    double[] gain = new double[3];
    for (int axis = 0; axis < 3; axis++) {
      variance[axis] += drift[axis];
      gain[axis] = variance[axis] / (variance[axis] + noise[axis]);
      variance[axis] *= 1.0 - gain[axis];
    }
    tagPose =
        new Pose2d(
            tagPose.getX() + gain[0] * dx,
            tagPose.getY() + gain[1] * dy,
            tagPose.getRotation().plus(Rotation2d.fromRadians(gain[2] * dYaw)));
    goodFrames++;
    lastSeenTime = Utils.getCurrentTimeSeconds();
  }

  private void restart(Pose2d measured, double[] noise) {
    tagPose = measured;
    System.arraycopy(noise, 0, variance, 0, 3);
    goodFrames = 1;
    jumpsInARow = 0;
    lastSeenTime = Utils.getCurrentTimeSeconds();
  }
}
