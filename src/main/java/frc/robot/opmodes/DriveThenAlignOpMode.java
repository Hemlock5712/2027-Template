// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.opmodes;

import static org.wpilib.units.Units.Seconds;

import frc.robot.Robot;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.DriveToTag;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.PeriodicOpMode;

/**
 * Global pose to get close, the tag itself for the last bit, with the arm moving the whole time.
 * Written as a <b>coroutine</b> (advanced - see ONBOARDING "Holds never finish"):
 *
 * <ul>
 *   <li>{@code fork} starts a command and keeps going without waiting.
 *   <li>{@code await} starts a command and waits for it to finish.
 * </ul>
 *
 * <p>Forked commands stop when the routine ends, so forking a hold here is safe.
 */
@Autonomous(name = "4 - Drive Then Align")
public class DriveThenAlignOpMode extends PeriodicOpMode {
  // TODO: the real tag for the scoring spot (it differs per alliance).
  private static final int TAG_ID = 1;
  // Where to stop, in the tag's frame. TODO: measure with the "Teach Tag Goal" utility OpMode.
  private static final Transform2d TAG_TO_GOAL =
      new Transform2d(0.6, 0.0, Rotation2d.fromDegrees(180));

  private final Command routine;

  public DriveThenAlignOpMode(Robot robot) {
    // Blue-origin point about 1 m in front of the target, and where the field map puts the
    // scoring spot (only used to shape the corner). TODO: real poses.
    final Pose2d staging = new Pose2d(2.0, 1.0, Rotation2d.fromDegrees(0));
    final Pose2d scoringGuess = new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0));

    routine =
        Command.noRequirements(
                coroutine -> {
                  // The arm rises while we drive.
                  coroutine.fork(robot.arm.scoring());
                  // Global: pass through the staging point toward the scoring spot, no stop.
                  coroutine.await(
                      new DriveToPose(robot.drivetrain, staging).passingThrough(scoringGuess));
                  // Local: lock onto the tag and finish on it.
                  coroutine.await(
                      new DriveToTag(
                              robot.drivetrain,
                              TAG_ID,
                              TAG_TO_GOAL,
                              robot.limelightBR,
                              robot.limelightBL)
                          .withTimeout(Seconds.of(3.0)));
                  coroutine.await(robot.flywheel.spinUp().withTimeout(Seconds.of(1.0)));
                })
            .named("Drive Then Align");
  }

  @Override
  public void start() {
    Scheduler.getDefault().schedule(routine);
  }

  @Override
  public void end() {
    Scheduler.getDefault().cancel(routine);
  }
}
