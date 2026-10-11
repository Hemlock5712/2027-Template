// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import com.ctre.phoenix6.Utils;
import com.limelightvision.DetectorTarget;
import com.limelightvision.Limelight;
import com.limelightvision.LimelightResults;
import frc.robot.subsystems.DriveMechanism;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;

/**
 * Tracks one game piece from a Limelight's neural detector, as a point on the floor in the
 * wheel-odometry frame - the game-piece version of {@link TagTracker}.
 *
 * <p>Each detection's angle is projected down to the floor using the camera's mounting pose (set in
 * the Limelight web UI), then placed with the odometry pose from when the picture was taken.
 * Distance from a floor projection is much weaker than direction - a 1 degree camera-pitch error is
 * several cm at 1 m - so give the intake room to absorb it.
 */
public class PieceTracker {
  // Height of the piece's center above the carpet. TODO: set for the real game piece.
  private static final double PIECE_HEIGHT_METERS = 0.05;
  private static final double MIN_CONFIDENCE = 0.5;
  // Loose on purpose: a slightly wrong camera pitch makes floor range read meters long.
  private static final double MAX_DISTANCE_METERS = 8.0;
  private static final double MAX_FRAME_AGE_SECONDS = 0.25;
  // A detection farther than this from the estimate is a different piece.
  private static final double SAME_PIECE_METERS = 0.4;
  // How much each new frame moves the estimate (0 = never, 1 = jump straight to it).
  private static final double SMOOTHING = 0.3;
  private static final int FRAMES_TO_LOCK = 3;

  private final DriveMechanism drivetrain;
  private final Limelight camera;
  private final int classId;

  private long lastFrameIndex = -1;
  private Translation2d piece; // odometry frame; null until seen
  private int goodFrames;

  /**
   * @param camera the object-detection camera
   * @param classId the detector class to chase, or -1 for any
   */
  public PieceTracker(DriveMechanism drivetrain, Limelight camera, int classId) {
    this.drivetrain = drivetrain;
    this.camera = camera;
    this.classId = classId;
  }

  /** Forgets the piece. Call when starting a new approach. */
  public void reset() {
    piece = null;
    goodFrames = 0;
    lastFrameIndex = -1;
  }

  /** Reads the newest frame and folds the matching detection into the estimate. */
  public void update() {
    LimelightResults frame = camera.getLatestResults();
    if (frame.frameIndex == lastFrameIndex) {
      return; // nothing new
    }
    lastFrameIndex = frame.frameIndex;

    double frameAge =
        frame.getAgeSeconds()
            + (frame.captureLatencyMillis + frame.targetingLatencyMillis) / 1000.0;
    if (frameAge > MAX_FRAME_AGE_SECONDS) {
      return;
    }
    Optional<Pose2d> robotAtCapture =
        drivetrain.getOdometryPoseAt(Utils.getCurrentTimeSeconds() - frameAge);
    if (robotAtCapture.isEmpty()) {
      return;
    }
    Pose3d cameraInRobot = frame.getCameraPose_RobotSpace();

    // Keep following the same piece: pick the detection nearest our estimate (or the robot).
    Translation2d best = null;
    double bestScore = Double.POSITIVE_INFINITY;
    for (DetectorTarget target : frame.detectorTargets) {
      if (target.confidence < MIN_CONFIDENCE || (classId >= 0 && target.classId != classId)) {
        continue;
      }
      Optional<Translation2d> inRobot =
          floorPoint(cameraInRobot, target.txDegreesNoCrosshair, target.tyDegreesNoCrosshair);
      if (inRobot.isEmpty() || inRobot.get().getNorm() > MAX_DISTANCE_METERS) {
        continue;
      }
      Pose2d robot = robotAtCapture.get();
      Translation2d inOdometry =
          robot.getTranslation().plus(inRobot.get().rotateBy(robot.getRotation()));
      double score = piece == null ? inRobot.get().getNorm() : inOdometry.getDistance(piece);
      if (score < bestScore) {
        best = inOdometry;
        bestScore = score;
      }
    }
    if (best == null) {
      return;
    }

    if (piece == null || best.getDistance(piece) > SAME_PIECE_METERS) {
      piece = best; // first sighting, or our piece is gone and this is a new one
      goodFrames = 1;
    } else {
      piece = piece.interpolate(best, SMOOTHING);
      goodFrames++;
    }
    Logger.recordOutput("PieceTracker/PieceInOdometry", new Pose2d(piece, new Rotation2d()));
    Logger.recordOutput("PieceTracker/Locked", hasLock());
  }

  /** True once a few frames agree. Stays true while odometry coasts (piece under the bumper). */
  public boolean hasLock() {
    return piece != null && goodFrames >= FRAMES_TO_LOCK;
  }

  /**
   * The piece as a field position the drivetrain can drive to - measured from where the robot is
   * now (wheel odometry), then placed onto the field pose, like {@link TagTracker#getFieldGoal}.
   *
   * @return empty until {@link #hasLock()}
   */
  public Optional<Translation2d> getFieldPosition() {
    if (!hasLock()) {
      return Optional.empty();
    }
    Transform2d robotToPiece =
        new Pose2d(piece, new Rotation2d()).minus(drivetrain.getOdometryPose());
    return Optional.of(drivetrain.getPose().transformBy(robotToPiece).getTranslation());
  }

  /**
   * Where the camera's line of sight to a detection hits the plane at piece height, in the robot
   * frame. Empty if it never hits (looking level or up).
   */
  private static Optional<Translation2d> floorPoint(
      Pose3d cameraInRobot, double txDegrees, double tyDegrees) {
    // Camera frame: +X out of the lens, +Y left, +Z up. tx is positive to the RIGHT, ty UP.
    // TODO: confirm both signs on the robot - a piece on the left must log a positive y.
    Translation3d ray =
        new Translation3d(
                1.0, -Math.tan(Math.toRadians(txDegrees)), Math.tan(Math.toRadians(tyDegrees)))
            .rotateBy(cameraInRobot.getRotation());
    if (ray.getZ() > -1e-6) {
      return Optional.empty();
    }
    double scale = (PIECE_HEIGHT_METERS - cameraInRobot.getZ()) / ray.getZ();
    return Optional.of(cameraInRobot.getTranslation().plus(ray.times(scale)).toTranslation2d());
  }
}
