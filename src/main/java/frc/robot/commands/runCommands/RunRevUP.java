// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.runCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class RunRevUP extends Command {
  /** Creates a new RunRevUP. */
  private final Shooter m_RunRevUp;
  private final double value;
  public RunRevUP(Shooter shooter, double value) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_RunRevUp = shooter;
    this.value = value;
    addRequirements(m_RunRevUp);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_RunRevUp.idleFire(value);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_RunRevUp.fireStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
