// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Trolley;
import frc.robot.subsystems.Wrist;

public class AutoIntake extends SequentialCommandGroup {
  /** Creates a new AutoIntake. */
  public AutoIntake(Intake intake, Wrist wrist, Trolley trolley) {
    addCommands(
      new RunCommand(() -> wrist.setPosition(WristConstants.INTAKE_SETPOINT_POS), wrist)
        .raceWith(new RunIntake(intake, 0.3)).until(intake::hasNote),
        // TODO: Should probably be alongWith?
      new RunCommand(() -> wrist.setPosition(WristConstants.SHOOTING_SETPOINT_POS), wrist)
    );
  }
}
