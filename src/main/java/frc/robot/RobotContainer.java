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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunPivot;
import frc.robot.commands.RunShooter;
import frc.robot.commands.RunTrolley;
import frc.robot.commands.RunWrist;
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
  private final Shooter shooter = new Shooter();
  private final LEDStrip leds = new LEDStrip();

  // The robot's subsystems and commands are defined here...
  // private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
  //                                                                        "swerve/neo"));

  PS5Controller roller = new PS5Controller(0);

  private final JoystickButton INTAKE = new JoystickButton(roller, PS5Controller.Button.kCross.value);
  private final JoystickButton AUTO_INTAKE = new JoystickButton(roller, PS5Controller.Button.kCircle.value);
  private final JoystickButton WRIST_IN = new JoystickButton(roller, PS5Controller.Button.kR1.value);
  private final JoystickButton WRIST_OUT = new JoystickButton(roller, PS5Controller.Button.kL1.value);
  private final JoystickButton TROLLEY_IN = new JoystickButton(roller, PS5Controller.Button.kR3.value);
  private final JoystickButton TROLLEY_OUT = new JoystickButton(roller, PS5Controller.Button.kL3.value);
  private final JoystickButton PIVOT_IN = new JoystickButton(roller, PS5Controller.Button.kR2.value);
  private final JoystickButton PIVOT_OUT = new JoystickButton(roller, PS5Controller.Button.kL2.value);
  private final JoystickButton REV_SHOOTER = new JoystickButton(roller, PS5Controller.Button.kSquare.value);
  private final JoystickButton GET_LITTT = new JoystickButton(roller, PS5Controller.Button.kTouchpad.value);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // leds.setDefaultCommand(new RunCommand(() -> leds.noLED(), leds));
    // Configure the trigger bindings
    configureBindings();
    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    // Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
    //     () -> MathUtil.applyDeadband(roller.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    //     () -> MathUtil.applyDeadband(roller.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
    //     () -> roller.getRightX(),
    //     () -> roller.getRightY());

    // // Applies deadbands and inverts controls because joysticks
    // // are back-right positive while robot
    // // controls are front-left positive
    // // left stick controls translation
    // // right stick controls the angular velocity of the robot
    // Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
    //     () -> MathUtil.applyDeadband(roller.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    //     () -> MathUtil.applyDeadband(roller.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
    //     () -> roller.getRawAxis(2));

    // Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
    //     () -> -MathUtil.applyDeadband(roller.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    //     () -> -MathUtil.applyDeadband(roller.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
    //     () -> -roller.getRawAxis(2));

    // drivebase.setDefaultCommand(
    //     !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
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

    GET_LITTT.whileTrue(new RunCommand(() -> leds.royalBlueLED()));
    INTAKE.whileTrue(new RunIntake(intake, 0.3).alongWith(new RunCommand(() -> leds.redLED())));
    AUTO_INTAKE.whileTrue(new RunIntake(intake, 0.3).until(intake::hasNote));
    WRIST_IN.whileTrue(new RunWrist(wrist, 0.3));
    WRIST_OUT.whileTrue(new RunWrist(wrist, -0.3));
    TROLLEY_IN.whileTrue(new RunTrolley(trolley, 0.4));
    TROLLEY_OUT.whileTrue(new RunTrolley(trolley, -0.4));
    PIVOT_IN.whileTrue(new RunPivot(pivot, 0.2));
    PIVOT_OUT.whileTrue(new RunPivot(pivot, -0.2));
    REV_SHOOTER.whileTrue(new RunShooter(shooter, 0.9));

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // new JoystickButton(roller, 1).onTrue((new InstantCommand(drivebase::zeroGyro)));
    // // new JoystickButton(roller, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    // new JoystickButton(roller,
    //                    2).whileTrue(
    //     Commands.deferredProxy(() -> drivebase.driveToPose(
    //                                new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
    //                           ));
    // new JoystickButton(roller, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    // return drivebase.getAutonomousCommand("New Path", true);
    return null;
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    // drivebase.setMotorBrake(brake);
  }
}
