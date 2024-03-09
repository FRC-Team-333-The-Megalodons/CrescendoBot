// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manualCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Wrist;

public class wristIntakePos extends Command {
  /** Creates a new wristAtSetpoint. */
  private final Wrist m_Wrist;
  //private final double value;
  public wristIntakePos(Wrist wrist) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Wrist = wrist;
    //this.value = value;
    addRequirements(m_Wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Wrist.wristController(WristConstants.NEW_INTAKE_POS);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Wrist.wristSTOP();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
