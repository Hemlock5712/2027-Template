// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.opmodes;

import frc.robot.Robot;
import frc.robot.subsystems.vision.TagTracker;
import org.littletonrobotics.junction.Logger;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Utility;

/**
 * Measures a {@code tagToGoal} for {@link frc.robot.commands.DriveToTag}. Push the robot by hand to
 * exactly where it should score on a real field element, enable this, wait a few seconds, then copy
 * the printed line into your code.
 *
 * <p>It uses the same cameras and math as DriveToTag, so camera-mounting errors cancel out.
 */
@Utility(name = "Teach Tag Goal")
public class TeachTagGoalOpMode extends PeriodicOpMode {
  // TODO: the tag on the element you are teaching.
  private static final int TAG_ID = 1;

  private final TagTracker tracker;
  private int loops;

  public TeachTagGoalOpMode(Robot robot) {
    tracker = new TagTracker(robot.drivetrain, TAG_ID, robot.limelightBR, robot.limelightBL);
  }

  @Override
  public void start() {
    tracker.reset();
    loops = 0;
  }

  @Override
  public void periodic() {
    tracker.update();
    tracker
        .getRobotInTag()
        .ifPresent(
            robotInTag -> {
              Logger.recordOutput("TeachTagGoal/RobotInTag", robotInTag);
              if (++loops % 50 == 0) { // about once a second
                System.out.printf(
                    "Tag %d: new Transform2d(%.3f, %.3f, Rotation2d.fromDegrees(%.1f))%n",
                    TAG_ID,
                    robotInTag.getX(),
                    robotInTag.getY(),
                    robotInTag.getRotation().getDegrees());
              }
            });
  }
}
