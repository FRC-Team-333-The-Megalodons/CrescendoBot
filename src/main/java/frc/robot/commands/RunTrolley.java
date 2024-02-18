// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Trolley;

public class RunTrolley extends Command {
  /** Creates a new RunTrolley. */
  private Trolley mTrolley;
  private double setpoint;
  public RunTrolley(Trolley mTrolley, double setpoint) {
    this.mTrolley = mTrolley;
    this.setpoint = setpoint;
    addRequirements(mTrolley);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mTrolley.trolleyTo(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mTrolley.stopTrolly();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
