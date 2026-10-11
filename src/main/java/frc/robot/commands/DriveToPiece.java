// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.limelightvision.Limelight;
import frc.robot.subsystems.DriveMechanism;
import frc.robot.subsystems.vision.PieceTracker;
import java.util.Optional;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * Drive up to a game piece, facing it, with the intake on top of it. Uses {@link DriveToPose} with
 * the piece from a {@link PieceTracker}; holds still until a piece is locked.
 *
 * <p>End it with your intake's sensor at the call site: {@code new DriveToPiece(...).until(intake::
 * hasPiece)}.
 */
public class DriveToPiece extends DriveToPose {
  // Robot center stops this far behind the piece. TODO: set from the real intake.
  private static final double INTAKE_REACH_METERS = 0.45;

  private final DriveMechanism drivetrain;
  private final PieceTracker tracker;
  private Pose2d holdPose;

  /**
   * @param camera the object-detection camera
   * @param classId the detector class to chase, or -1 for any
   */
  public DriveToPiece(DriveMechanism drivetrain, Limelight camera, int classId) {
    super("DriveToPiece", drivetrain);
    this.drivetrain = drivetrain;
    this.tracker = new PieceTracker(drivetrain, camera, classId);
  }

  @Override
  protected void initialize() {
    tracker.reset();
    holdPose = drivetrain.getPose();
    super.initialize();
  }

  @Override
  protected void execute() {
    tracker.update();
    super.execute();
  }

  @Override
  protected Pose2d goal() {
    Optional<Translation2d> piece = tracker.getFieldPosition();
    if (piece.isEmpty()) {
      return holdPose;
    }
    Translation2d toPiece = piece.get().minus(drivetrain.getPose().getTranslation());
    if (toPiece.getNorm() < 1e-3) {
      return drivetrain.getPose();
    }
    Rotation2d facing = new Rotation2d(toPiece.getX(), toPiece.getY());
    return new Pose2d(piece.get().minus(new Translation2d(INTAKE_REACH_METERS, facing)), facing);
  }

  @Override
  protected boolean isFinished() {
    return tracker.hasLock() && super.isFinished();
  }
}
