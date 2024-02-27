// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manualCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.test;

public class testtest extends Command {
  /** Creates a new testtest. */
  private final test m_test;
  
  public testtest(test m_Test) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_test = m_Test;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_test.testIN();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_test.testSTOP();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
