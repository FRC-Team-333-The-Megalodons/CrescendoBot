// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.AbsoluteDriveAdv;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  private final CommandXboxController driverRoller = new CommandXboxController(0);
  private final XboxController ps5Controller = new XboxController(0);

  public RobotContainer() {
    configureBindings();

    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> MathUtil.applyDeadband(driverRoller.getLeftY(),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> MathUtil.applyDeadband(driverRoller.getLeftX(),
                                                                                                OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> MathUtil.applyDeadband(driverRoller.getRightX(),
                                                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                                   ps5Controller::getYButtonPressed,
                                                                   ps5Controller::getBackButtonPressed,
                                                                   ps5Controller::getXButtonPressed,
                                                                   ps5Controller::getBButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverRoller.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverRoller.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverRoller.getRightX(),
        () -> driverRoller.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverRoller.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverRoller.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverRoller.getRawAxis(2));

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driverRoller.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverRoller.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverRoller.getRawAxis(2));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
  }

  private void configureBindings() {

    driverRoller.leftBumper().onTrue(new InstantCommand(drivebase::zeroGyro));
    driverRoller.rightBumper().onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    driverRoller.a().whileTrue(
      Commands.deferredProxy(() -> drivebase.driveToPose(new Pose2d(new Translation2d(4,4), Rotation2d.fromDegrees(0))))
    );
  }

  public Command getAutonomousCommand() {
    return null;
  }
}