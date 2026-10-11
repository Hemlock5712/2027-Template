// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.opmodes;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.Robot;
import frc.robot.commands.DriveToPiece;
import frc.robot.commands.DriveToTag;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.DriveMechanism;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

/**
 * Driver teleop - the OpMode replacement for {@code RobotContainer.configureBindings}. Bindings
 * made in the constructor are scoped to this OpMode and removed on a mode switch. Add another
 * {@code @Teleop} class for a second driver layout.
 */
@Teleop(name = "Teleop")
public class TeleopOpMode extends PeriodicOpMode {
  // Which AprilTag to auto-align to. TODO: pick the real tag ID (it differs per alliance) once
  // the game is wired - see game-info.
  private static final int ALIGN_TAG_ID = 1;
  // Where to stop, in the tag's frame: 0.6 m out from the tag face, facing it. TODO: the real
  // one from CAD; run the "Calibrate Cameras" utility OpMode on that spot.
  private static final Transform2d ALIGN_TAG_TO_GOAL =
      new Transform2d(0.6, 0.0, Rotation2d.fromDegrees(180));

  private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(maxSpeed * 0.1)
          .withRotationalDeadband(maxAngularRate * 0.1) // 10% stick deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final CommandNiDsXboxController driver = new CommandNiDsXboxController(0);

  public TeleopOpMode(Robot robot) {
    final DriveMechanism drivetrain = robot.drivetrain;

    // WPILib axes: X is forward, Y is left - hence the minus signs on the sticks.
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-driver.getLeftY() * maxSpeed)
                    .withVelocityY(-driver.getLeftX() * maxSpeed)
                    .withRotationalRate(-driver.getRightX() * maxAngularRate)));

    // Superstructure presets (arm + flywheel move together), held while the button is down.
    driver.leftTrigger().whileTrue(robot.intake());
    driver.rightBumper().whileTrue(robot.score());
    driver.rightTrigger().whileTrue(robot.stow());

    // Hold A: drive to the spot in front of the tag - field map first, then the tag itself.
    driver
        .a()
        .whileTrue(
            new DriveToTag(
                drivetrain, ALIGN_TAG_ID, ALIGN_TAG_TO_GOAL, robot.limelightBR, robot.limelightBL));

    // Hold B: drive onto the nearest game piece. TODO: use your object-detection camera.
    driver.b().whileTrue(new DriveToPiece(drivetrain, robot.limelightBL, -1));

    // Hold Y: auto-score prep. Letting go stops the flywheel via its default command, so no
    // "stop" binding is needed here.
    driver.y().whileTrue(robot.autoScore());
  }
}
