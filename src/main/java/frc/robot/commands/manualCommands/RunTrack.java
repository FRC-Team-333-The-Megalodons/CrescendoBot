// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manualCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Trolley;

public class RunTrack extends Command {
  /** Creates a new RunTrack. */
  private final Trolley m_Track;
  private final double value;
  public RunTrack(Trolley track, double value) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Track = track;
    this.value = value;
    addRequirements(m_Track);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Track.trolley(value);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Track.getLimitSwitch();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
