// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.imageio.plugins.jpeg.JPEGHuffmanTable;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.simulation.PS5ControllerSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.IntakeManual;
import frc.robot.commands.IntakeSensors;
import frc.robot.commands.Ogon;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Track;
import frc.robot.subsystems.Wrist;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Intake m_Intake = new Intake();
  private final Shooter m_Shooter = new Shooter();
  private final Pivot m_Pivot = new Pivot();
  private final Track m_Track = new Track();
  private final Wrist m_Wrist = new Wrist();
  private final LEDs m_Leds = new LEDs();
  private final Joystick joy = new Joystick(0);


  private final JoystickButton INTAKE_MANUAL_BUTTON = new JoystickButton(joy, PS5Controller.Button.kR1.value);
  private final JoystickButton INTAKE_SENSORS_BUTTON  = new JoystickButton(joy, PS5Controller.Button.kL1.value); 
  private final JoystickButton FIRE_BUTTON = new JoystickButton(joy, PS5Controller.Button.kCircle.value);
  private final JoystickButton IDLE_FIRE_BUTTON = new JoystickButton(joy, PS5Controller.Button.kTouchpad.value);
  private final JoystickButton TRACK_UP = new JoystickButton(joy, PS5Controller.Button.kR3.value);
  private final JoystickButton TRACK_DOWN = new JoystickButton(joy, PS5Controller.Button.kL3.value);
  private final JoystickButton WRIST_UP = new JoystickButton(joy, PS5Controller.Button.kCross.value);
  private final JoystickButton WRIST_DOWN = new JoystickButton(joy, PS5Controller.Button.kCircle.value);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_Intake.setDefaultCommand(new RunCommand(() -> m_Intake.intakeStop(), m_Intake));
    m_Shooter.setDefaultCommand(new RunCommand(() -> m_Shooter.fireStop(), m_Shooter));
    m_Wrist.setDefaultCommand(new RunCommand(() -> m_Wrist.wristSTOP(), m_Wrist));
    m_Track.setDefaultCommand(new RunCommand(() -> m_Track.trackStop(), m_Track));
    m_Leds.setDefaultCommand(new RunCommand(() -> m_Leds.royalBlueLED(), m_Leds));
    configureButtonBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.  ..
   */

   private void configureButtonBindings() {
    INTAKE_SENSORS_BUTTON.whileTrue(new IntakeSensors(m_Intake));  
    INTAKE_MANUAL_BUTTON.whileTrue(new IntakeManual(m_Intake));
    FIRE_BUTTON.whileTrue(new Ogon(m_Shooter));
    IDLE_FIRE_BUTTON.whileTrue(new RunCommand(() -> m_Shooter.idleFire()));
    TRACK_UP.whileTrue(new RunCommand(() -> m_Track.trackUp()));
    TRACK_DOWN.whileTrue(new RunCommand(() -> m_Track.trackDown()));
    WRIST_UP.whileTrue(new RunCommand(() -> m_Wrist.wristUP()));
    WRIST_DOWN.whileTrue(new RunCommand(() -> m_Wrist.wristDOWN()));
  }
  /** mxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
