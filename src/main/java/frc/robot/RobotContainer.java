// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TrolleyConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.RunIndexer;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunPivot;
import frc.robot.commands.RunShooter;
import frc.robot.commands.RunTrolley;
import frc.robot.commands.RunWrist;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDStrip;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Trolley;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  private final Intake intake = new Intake();
  private final Wrist wrist = new Wrist();
  private final Trolley trolley = new Trolley();
  private final Pivot pivot = new Pivot();
  private final Indexer indexer = new Indexer();
  private final Shooter shooter = new Shooter();
  private final LEDStrip leds = new LEDStrip();
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));

  CommandPS5Controller driverRoller = new CommandPS5Controller(0);
  CommandPS5Controller opRoller = new CommandPS5Controller(1);
  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(driverRoller.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverRoller.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverRoller.getRightX());
        //() -> driverRoller.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(driverRoller.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverRoller.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverRoller.getRightX());

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> -MathUtil.applyDeadband(driverRoller.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverRoller.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverRoller.getRightX());

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    boolean manualMode = true;

    if (manualMode) {
      opRoller.circle().whileTrue(new RunIntake(intake, 0.3).until(intake::hasNote));
      opRoller.cross().whileTrue(new RunIntake(intake, 1.0));
      opRoller.triangle().whileTrue(new RunIntake(intake, -1.0));

      opRoller.touchpad().whileTrue(new RunCommand(() -> leds.royalBlueLED()));

      opRoller.L1().whileTrue(new RunWrist(wrist, -0.2));
      opRoller.R1().whileTrue(new RunWrist(wrist, 0.2));
      // opRoller.L3().whileTrue(new RunCommand(() -> wrist.setPosition(WristConstants.SHOOTING_SETPOINT), wrist));
      // opRoller.R3().whileTrue(new RunCommand(() -> wrist.setPosition(WristConstants.INTAKE_SETPOINT), wrist));

      opRoller.povUp().whileTrue(new RunTrolley(trolley, 1.0));
      opRoller.povDown().whileTrue(new RunTrolley(trolley, -1.0));
      // opRoller.povDownRight().whileTrue(new RunCommand(() -> trolley.setPosition(TrolleyConstants.INTAKE_SETPOINT), trolley));

      opRoller.R2().whileTrue(new RunPivot(pivot, 0.2));
      opRoller.L2().whileTrue(new RunPivot(pivot, -0.2));
      // opRoller.R3().whileTrue(new RunCommand(() -> pivot.setAngle(PivotConstants.INTAKE_SETPOINT), pivot));
      // opRoller.L3().whileTrue(new RunCommand(() -> pivot.setAngle(PivotConstants.HOME_SETPOINT), pivot));

      opRoller.square().whileTrue(new RunShooter(shooter, 0.9).alongWith(new RunIndexer(indexer, 0.45)));
    } else {
      opRoller.circle().whileTrue(new RunCommand(() -> wrist.setPosition(WristConstants.INTAKE_SETPOINT), wrist).raceWith(new RunIntake(intake, 0.3).until(intake::hasNote)).andThen(new RunCommand(() -> wrist.setPosition(WristConstants.SHOOTING_SETPOINT), wrist)));
      // opRoller.circle().whileTrue(new AutoIntake(intake, wrist, trolley));

      // opRoller.L1().whileTrue(new RunCommand(() -> wrist.setPosition(WristConstants.SHOOTING_SETPOINT), wrist));
      // opRoller.R1().whileTrue(new RunCommand(() -> wrist.setPosition(WristConstants.INTAKE_SETPOINT), wrist));

      // opRoller.povUp().whileTrue(new RunCommand(() -> trolley.setPosition(TrolleyConstants.INTAKE_SETPOINT), trolley));
      opRoller.square().whileTrue(new RunCommand(() -> shooter.setSpeed(ShooterConstants.SHOT_RPM), shooter).alongWith(new RunCommand(() -> indexer.setSpeed(IndexerConstants.SHOT_RPM), indexer)));
    }

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    driverRoller.cross().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverRoller.square().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    driverRoller.circle().whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(
                                   new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              ));
    // driverRoller.triangle().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setDriveMode()
  {
    // drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
