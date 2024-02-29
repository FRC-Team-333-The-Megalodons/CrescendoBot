// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automatedCommands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.manualCommands.RunIntakeSensors;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Trolley;
import frc.robot.subsystems.Wrist;

public class AutoIntake extends SequentialCommandGroup {
  /** Creates a new AutoIntake. */

  public AutoIntake(Intake m_Intake, Wrist m_Wrist, Trolley m_Trolley) {
    addCommands(
      new RunCommand(() -> m_Wrist.setPosition(WristConstants.INTAKE_SETPOINT), m_Wrist)
        .raceWith(new RunIntakeSensors(m_Intake, 0.3).until(m_Intake::detectNote)),
      new RunCommand(() -> m_Wrist.setPosition(WristConstants.SHOOTING_SETPOINT), m_Wrist)
    );
  }
}
