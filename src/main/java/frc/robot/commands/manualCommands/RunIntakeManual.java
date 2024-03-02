// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manualCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;

public class RunIntakeManual extends Command {
  private final Intake m_IntakeManual;
  private final LEDs m_Leds;
  private final double value;

  public RunIntakeManual(Intake intake, double value, LEDs leds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_IntakeManual = intake;
    this.m_Leds = leds;
    this.value = value;
    addRequirements(m_IntakeManual);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeManual.intake(value);
    m_Leds.blinkingColor(100, 0, 255);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeManual.intakeStop();
    m_Leds.noLED();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}