// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.TrolleyConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Trolley;
import frc.robot.subsystems.Wrist;

public class AutoAmp extends SequentialCommandGroup {
  /** Creates a new AutoAmp. */
  public AutoAmp(Intake intake, Wrist wrist, Trolley trolley) {
    addCommands(
      new RunCommand(() -> trolley.trolleyToSetpoint(TrolleyConstants.AMP_SETPOINT), trolley),
      new RunCommand(() -> wrist.wristToSetpoint(WristConstants.AMP_SETPOINT), wrist),
      new RunIntake(intake, -0.3)
    );
  }
}
