// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import com.ctre.phoenix6.Utils;
import com.limelightvision.FiducialTarget;
import com.limelightvision.Limelight;
import com.limelightvision.LimelightResults;
import frc.robot.subsystems.DriveMechanism;
import frc.robot.utils.AllianceFlip;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.linalg.MatBuilder;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N2;
import org.wpilib.math.util.Nat;

/**
 * Tracks where one AprilTag really is, as a pose in the wheel-odometry frame ({@link
 * DriveMechanism#getOdometryPose()}). This is the "local pose": a misplaced field element or a jump
 * in the vision-corrected field pose can't pull alignment off.
 *
 * <p>Each frame is placed using the odometry pose from the moment the picture was taken (latency
 * compensation), corrected for its camera ({@link CameraCorrections}), and filtered. When the tag
 * leaves view - too close, blocked - odometry carries the estimate the rest of the way.
 *
 * <p>Tuned in tools/alignsim (docs/alignment/studies/tag-tracking.md). Call {@link #update()} every
 * loop while you use it ({@link frc.robot.commands.DriveToTag} does).
 */
public class TagTracker {
  // Frames outside these limits are skipped. TODO: tune from logs.
  private static final double MAX_TAG_DISTANCE_METERS = 3.5;
  private static final double MAX_FRAME_AGE_SECONDS = 0.25;
  // Once locked, a frame this far off is a bad solve - unless 3 in a row agree (then restart).
  private static final double JUMP_METERS = 0.25;
  private static final int JUMPS_TO_RESTART = 3;
  // A frame whose yaw is this many sigmas off (a flipped solve) still updates x/y, but not yaw.
  private static final double YAW_GATE_SIGMAS = 4.0;
  // Locked = this many frames AND x/y known to within LOCK_SIGMA.
  private static final int FRAMES_TO_LOCK = 3;
  private static final double LOCK_SIGMA_METERS = 0.03;

  // Per-frame noise (1 sigma) at distance d. TODO: fit from logs of the robot parked at 1, 2, 3 m.
  private static final double NOISE_FLOOR = 0.004; // m
  private static final double DEPTH_NOISE_PER_M2 = 0.006; // along the camera ray: floor + k*d^2
  private static final double SIDE_NOISE_PER_M = 0.003; // across the ray: floor + k*d
  private static final double YAW_NOISE_FLOOR = Math.toRadians(0.3);
  private static final double YAW_NOISE_PER_M = Math.toRadians(0.45);
  private static final double YAW_NOISE_PER_AMBIGUITY = Math.toRadians(9.0); // head-on views flip
  // How far the tag may appear to move per frame (odometry drift); keeps the filter listening.
  private static final double XY_DRIFT = 0.0015; // m
  private static final double YAW_DRIFT = Math.toRadians(0.05);
  // Starting guess: the field map's tag pose, trusted only this much.
  private static final double MAP_XY_SIGMA = 0.08; // m
  private static final double MAP_YAW_SIGMA = Math.toRadians(1.5);

  private final DriveMechanism drivetrain;
  private final int tagId;
  private final Limelight[] cameras;
  private final Transform2d[] corrections;
  private final long[] lastFrameIndex;
  private final String logPrefix;

  private Pose2d tagPose; // odometry frame; null until the map guess or first frame
  private Matrix<N2, N2> xyVariance; // x/y and yaw never mix, so they are filtered separately
  private double yawVariance;
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
    this.corrections =
        Arrays.stream(cameras).map(CameraCorrections::get).toArray(Transform2d[]::new);
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
    if (tagPose == null) {
      startFromMap();
    }
    List<Pose2d> tagInRobotLog = new ArrayList<>();
    List<Pose2d> robotAtCaptureLog = new ArrayList<>();
    List<Double> frameAgeLog = new ArrayList<>();
    List<Double> ambiguityLog = new ArrayList<>();
    List<Double> distanceLog = new ArrayList<>();
    List<Double> cameraLog = new ArrayList<>();

    for (int i = 0; i < cameras.length; i++) {
      LimelightResults frame = cameras[i].getLatestResults();
      if (frame.frameIndex == lastFrameIndex[i]) {
        continue; // nothing new from this camera
      }
      lastFrameIndex[i] = frame.frameIndex;

      FiducialTarget tag = findTag(frame);
      if (tag == null) {
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

      Pose2d rawTagInRobot = tagInRobot3d.toPose2d();
      // This camera's calibration fix first, then place the tag where the robot was.
      Pose2d tagInRobot =
          Pose2d.kZero.transformBy(corrections[i]).transformBy(asTransform(rawTagInRobot));
      Pose2d measured = robotAtCapture.get().transformBy(asTransform(tagInRobot));
      fuse(robotAtCapture.get(), measured, distance, tag.ambiguity);

      tagInRobotLog.add(rawTagInRobot);
      robotAtCaptureLog.add(robotAtCapture.get());
      frameAgeLog.add(frameAge);
      ambiguityLog.add(tag.ambiguity);
      distanceLog.add(distance);
      cameraLog.add((double) i);
    }

    // Raw inputs too, so other filters/strategies can be replayed offline
    // (tools/alignsim/replay.py).
    Logger.recordOutput(logPrefix + "TagInRobot", tagInRobotLog.toArray(Pose2d[]::new));
    Logger.recordOutput(logPrefix + "RobotAtCapture", robotAtCaptureLog.toArray(Pose2d[]::new));
    Logger.recordOutput(logPrefix + "FrameAgeSeconds", toArray(frameAgeLog));
    Logger.recordOutput(logPrefix + "Ambiguity", toArray(ambiguityLog));
    Logger.recordOutput(logPrefix + "Distance", toArray(distanceLog));
    Logger.recordOutput(logPrefix + "Camera", toArray(cameraLog));
    Logger.recordOutput(logPrefix + "Locked", hasLock());
    Logger.recordOutput(logPrefix + "SecondsSinceSeen", getSecondsSinceSeen());
    if (tagPose != null) {
      Logger.recordOutput(logPrefix + "TagPose", tagPose);
      Logger.recordOutput(logPrefix + "SigmaXYMeters", xySigma());
    }
  }

  /** True once a few frames agree on where the tag is. Stays true while odometry coasts. */
  public boolean hasLock() {
    return tagPose != null && goodFrames >= FRAMES_TO_LOCK && xySigma() < LOCK_SIGMA_METERS;
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

  /** Seconds since a frame last updated the estimate (infinite if never). */
  public double getSecondsSinceSeen() {
    return goodFrames == 0
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

  /** Seeds the estimate with the field map's tag pose, moved into the odometry frame. */
  private void startFromMap() {
    AllianceFlip.getTagPose(tagId)
        .ifPresent(
            tagOnField -> {
              Transform2d robotToTag = tagOnField.minus(drivetrain.getPose());
              tagPose = drivetrain.getOdometryPose().transformBy(robotToTag);
              xyVariance = Matrix.eye(Nat.N2()).times(MAP_XY_SIGMA * MAP_XY_SIGMA);
              yawVariance = MAP_YAW_SIGMA * MAP_YAW_SIGMA;
              Logger.recordOutput(logPrefix + "MapGuess", tagPose); // for offline replay
            });
  }

  /** Folds one measurement into the estimate - a small Kalman filter. */
  private void fuse(Pose2d robotAtCapture, Pose2d measured, double distance, double ambiguity) {
    // x/y noise is long along the camera ray (depth) and short across it.
    double depth = NOISE_FLOOR + DEPTH_NOISE_PER_M2 * distance * distance;
    double side = NOISE_FLOOR + SIDE_NOISE_PER_M * distance;
    double ray =
        Math.atan2(
            measured.getY() - robotAtCapture.getY(), measured.getX() - robotAtCapture.getX());
    double c = Math.cos(ray);
    double s = Math.sin(ray);
    double dd = depth * depth;
    double ss = side * side;
    Matrix<N2, N2> xyNoise =
        MatBuilder.fill(
            Nat.N2(),
            Nat.N2(),
            c * c * dd + s * s * ss,
            c * s * (dd - ss),
            c * s * (dd - ss),
            s * s * dd + c * c * ss);
    double yawSigma =
        YAW_NOISE_FLOOR + YAW_NOISE_PER_M * distance + YAW_NOISE_PER_AMBIGUITY * ambiguity;
    double yawNoise = yawSigma * yawSigma;

    if (tagPose == null) {
      restart(measured, xyNoise, yawNoise);
      return;
    }
    // The tag doesn't move, but odometry drifts a little between frames.
    xyVariance = xyVariance.plus(Matrix.eye(Nat.N2()).times(XY_DRIFT * XY_DRIFT));
    yawVariance += YAW_DRIFT * YAW_DRIFT;

    double dx = measured.getX() - tagPose.getX();
    double dy = measured.getY() - tagPose.getY();
    double dYaw = measured.getRotation().minus(tagPose.getRotation()).getRadians();
    if (Math.hypot(dx, dy) > JUMP_METERS && goodFrames >= FRAMES_TO_LOCK) {
      if (++jumpsInARow >= JUMPS_TO_RESTART) {
        restart(measured, xyNoise, yawNoise); // the old estimate was the wrong one
      }
      return;
    }
    jumpsInARow = 0;

    Matrix<N2, N2> gain = xyVariance.times(xyVariance.plus(xyNoise).inv());
    Matrix<N2, N1> xyStep = gain.times(VecBuilder.fill(dx, dy));
    xyVariance = Matrix.eye(Nat.N2()).minus(gain).times(xyVariance);

    double yawStep = 0.0;
    if (Math.abs(dYaw) < YAW_GATE_SIGMAS * Math.sqrt(yawVariance + yawNoise)) {
      double yawGain = yawVariance / (yawVariance + yawNoise);
      yawStep = yawGain * dYaw;
      yawVariance *= 1.0 - yawGain;
    }

    tagPose =
        new Pose2d(
            tagPose.getX() + xyStep.get(0, 0),
            tagPose.getY() + xyStep.get(1, 0),
            tagPose.getRotation().plus(Rotation2d.fromRadians(yawStep)));
    goodFrames++;
    lastSeenTime = Utils.getCurrentTimeSeconds();
  }

  private void restart(Pose2d measured, Matrix<N2, N2> xyNoise, double yawNoise) {
    tagPose = measured;
    xyVariance = xyNoise;
    yawVariance = yawNoise;
    goodFrames = 1;
    jumpsInARow = 0;
    lastSeenTime = Utils.getCurrentTimeSeconds();
  }

  private double xySigma() {
    return Math.sqrt(Math.max(xyVariance.get(0, 0), xyVariance.get(1, 1)));
  }

  private static Transform2d asTransform(Pose2d pose) {
    return new Transform2d(pose.getTranslation(), pose.getRotation());
  }

  private static double[] toArray(List<Double> values) {
    return values.stream().mapToDouble(Double::doubleValue).toArray();
  }
}
