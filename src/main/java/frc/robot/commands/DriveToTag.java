// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.limelightvision.Limelight;
import frc.robot.subsystems.DriveMechanism;
import frc.robot.subsystems.vision.TagTracker;
import frc.robot.utils.AllianceFlip;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Transform2d;

/**
 * Drive to a spot in front of an AprilTag, precisely. One controller ({@link DriveToPose}), two
 * goals:
 *
 * <ol>
 *   <li><b>Global</b> - until the cameras lock on, drive to where the field map says the spot is.
 *   <li><b>Local</b> - then drive to the spot measured from the tag itself ({@link TagTracker}), so
 *       a field element that isn't where the map says doesn't matter.
 * </ol>
 *
 * <p>Only finishes once locked; add {@code .withTimeout(...)} in autos in case the tag is never
 * seen.
 */
public class DriveToTag extends DriveToPose {
  private final DriveMechanism drivetrain;
  private final int tagId;
  private final Transform2d tagToGoal;
  private final TagTracker tracker;
  private Pose2d mapGoal;

  /**
   * @param tagId the AprilTag on the field element (tag IDs differ per alliance - pick the right
   *     one)
   * @param tagToGoal where the robot should stop, in the tag's frame: +X out of the tag face, +Y to
   *     the tag's left, rotation 180 degrees = facing the tag. Measure it with the "Teach Tag Goal"
   *     utility OpMode.
   * @param cameras every camera that might see the tag
   */
  public DriveToTag(
      DriveMechanism drivetrain, int tagId, Transform2d tagToGoal, Limelight... cameras) {
    super("DriveToTag", drivetrain);
    this.drivetrain = drivetrain;
    this.tagId = tagId;
    this.tagToGoal = tagToGoal;
    this.tracker = new TagTracker(drivetrain, tagId, cameras);
  }

  @Override
  protected void initialize() {
    tracker.reset();
    // Approach goal from the field map. No map entry: hold still until the cameras lock on.
    mapGoal =
        AllianceFlip.getTagPose(tagId)
            .map(tag -> tag.transformBy(tagToGoal))
            .orElse(drivetrain.getPose());
    super.initialize();
  }

  @Override
  protected void execute() {
    tracker.update();
    super.execute();
  }

  @Override
  protected Pose2d goal() {
    return tracker.getFieldGoal(tagToGoal).orElse(mapGoal);
  }

  @Override
  protected boolean isFinished() {
    return tracker.hasLock() && super.isFinished();
  }
}
