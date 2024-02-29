// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.manualCommands.RunFire;
import frc.robot.commands.manualCommands.RunIntakeManual;
import frc.robot.commands.manualCommands.RunIntakeSensors;
import frc.robot.commands.manualCommands.RunPivot;
import frc.robot.commands.manualCommands.RunRevUP;
import frc.robot.commands.manualCommands.RunTrack;
import frc.robot.commands.manualCommands.RunWrist;
import frc.robot.commands.manualCommands.wristIntakePos;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Trolley;
import frc.robot.subsystems.Wrist;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final Intake m_Intake = new Intake();
  // private final Shooter m_Shooter = new Shooter(); 
  // private final Pivot m_Pivot = new Pivot();
  // private final Trolley m_Track = new Trolley();
  // private final Wrist m_Wrist = new Wrist();
  private final CommandPS5Controller joy = new CommandPS5Controller(0);
  // private final Indexer m_Indexer = new Indexer();
//  private final Joystick joy = new Joystick(0);
  private final LEDs m_Leds = new LEDs();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
  //  m_Intake.setDefaultCommand(new RunCommand(() -> m_Intake.intakeStop(), m_Intake));
  //  m_Shooter.setDefaultCommand(new RunCommand(() -> m_Shooter.fireStop(), m_Shooter));
  //  m_Wrist.setDefaultCommand(new RunCommand(() -> m_Wrist.wristSTOP(), m_Wrist));
  //  m_Track.setDefaultCommand(new RunCommand(() -> m_Track.trolleyStop(), m_Track));
  //  m_Pivot.setDefaultCommand(new RunCommand(() -> m_Pivot.pivotStop(), m_Pivot));
  //  m_Indexer.setDefaultCommand(new RunCommand(() -> m_Indexer.indexStop(),  m_Indexer));
   //  m_Leds.setDefaultCommand(new RunCommand(() -> m_Leds.noLED(), m_Leds));
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
   * 
   */

   private void configureButtonBindings() {
    // joy.R1().whileTrue(new RunIntakeSensors(m_Intake, 0.3).until(m_Intake::detectNote));  // R1 intake
    // joy.L1().whileFalse(new RunIntakeSensors(m_Intake, 0));

    // joy.R2().whileTrue(new RunIntakeManual(m_Intake, 0.3));    // R2  push
    // joy.R2().whileFalse(new RunIntakeManual(m_Intake, 0));

    // joy.touchpad().whileTrue(new RunRevUP(m_Shooter, 0.3)); // TOUCH PAD rev up
    // joy.touchpad().whileFalse(new RunRevUP(m_Shooter, 0));

    // joy.circle().whileTrue(new RunFire(m_Shooter, -0.3, m_Indexer, -0.3));       //  CIRCLE shoot
    // joy.circle().whileFalse(new RunFire(m_Shooter, 0, m_Indexer, 0));
   
    // joy.R3().whileTrue(new RunTrack(m_Track, 0.3));    // R3       track up
    // joy.R3().whileFalse(new RunTrack(m_Track, 0));

    // joy.L3().whileTrue(new RunTrack(m_Track, -0.3)); // L3       track down
    // joy.L3().whileFalse(new RunTrack(m_Track, 0));

    // joy.cross().whileTrue(new RunWrist(m_Wrist, 0.3));    // CROSS    wrist  up
    // joy.cross().whileFalse(new RunWrist(m_Wrist, 0));

    // joy.square().whileTrue(new RunWrist(m_Wrist, -0.3));  // SQUARE wrist down
    // joy.square().whileFalse(new RunWrist(m_Wrist, 0));

    // joy.L1().whileTrue(new RunPivot(m_Pivot, 0.3));    // L1   pivot up
    // joy.L1().whileFalse(new RunPivot(m_Pivot, 0));

    // joy.L2().whileTrue(new RunPivot(m_Pivot, -0.3));  // L2   pivot down
    // joy.L2().whileFalse(new RunPivot(m_Pivot, 0));

    // joy.triangle().whileTrue(new wristIntakePos(m_Wrist, 0.15));
    // joy.triangle().whileFalse(new wristIntakePos(m_Wrist, 0));                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     


    joy.triangle().whileFalse((new RunCommand(() -> m_Leds.noLED(), m_Leds)));
    joy.triangle().whileTrue((new RunCommand(() -> m_Leds.blinkingColor(0, 0, 250), m_Leds))); //BLUE

    joy.circle().whileFalse((new RunCommand(() -> m_Leds.noLED(), m_Leds)));
    joy.circle().whileTrue((new RunCommand(() -> m_Leds.blinkingColor(250, 0, 0), m_Leds))); // RED
    
    joy.square().whileFalse((new RunCommand(() -> m_Leds.noLED(), m_Leds)));
    joy.square().whileTrue((new RunCommand(() -> m_Leds.blinkingColor(0, 250, 0), m_Leds))); // 

    joy.cross().whileFalse((new RunCommand(() -> m_Leds.noLED(), m_Leds)));
    joy.cross().whileTrue((new RunCommand(() -> m_Leds.blinkingColor(230, 57, 0), m_Leds)));

    joy.touchpad().whileFalse((new RunCommand(() -> m_Leds.noLED(), m_Leds)));
    joy.touchpad().whileTrue((new RunCommand(() -> m_Leds.blinkingColor(230, 223, 0), m_Leds)));
  }
  /** 
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
