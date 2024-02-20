// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.TrolleyConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Trolley;
import frc.robot.subsystems.Wrist;

public class GoHome extends SequentialCommandGroup {
  /** Creates a new GoHome. */
  public GoHome(Pivot pivot, Trolley trolley, Wrist wrist) {
    addCommands(
      new RunCommand(() -> pivot.setAngle(PivotConstants.HOME_SETPOINT), pivot),
      new RunCommand(() -> trolley.setPosition(TrolleyConstants.HOME_SETPOINT), trolley)
        .alongWith(new RunCommand(() -> wrist.setPosition(WristConstants.SHOOTING_SETPOINT), wrist))
    );
  }
}
