// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import com.limelightvision.Limelight;
import java.util.Map;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;

/**
 * One small fix per camera, measured by the "Calibrate Cameras" utility OpMode. It moves that
 * camera's tag measurements (robot frame) to where they really are, cancelling the mounting and
 * pose-solve errors that matter most for precise alignment. {@link TagTracker} applies it.
 */
public final class CameraCorrections {
  // TODO: run "Calibrate Cameras" and paste its printed lines here. Re-run after a camera moves.
  private static final Map<String, Transform2d> BY_CAMERA_NAME =
      Map.of(
          "limelight-br", new Transform2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
          "limelight-bl", new Transform2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));

  private CameraCorrections() {}

  /** The camera's correction, or no change if it hasn't been calibrated. */
  public static Transform2d get(Limelight camera) {
    return BY_CAMERA_NAME.getOrDefault(camera.getName(), new Transform2d());
  }
}
