// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private CANSparkFlex leftClimber, rightClimber;

  /** Creates a new Climber. */
  public Climber() {
    leftClimber = new CANSparkFlex(16, MotorType.kBrushless);
    rightClimber = new CANSparkFlex(17, MotorType.kBrushless);

    leftClimber.restoreFactoryDefaults();
    rightClimber.restoreFactoryDefaults();

    leftClimber.setIdleMode(IdleMode.kBrake);
    rightClimber.setIdleMode(IdleMode.kBrake);

    leftClimber.burnFlash();
    rightClimber.burnFlash();
  }

  public void runClimber(double value) {
    leftClimber.set(-value);
    rightClimber.set(value);
  }

  public void stopClimber() {
    leftClimber.set(0);
    rightClimber.set(0);
  }

  @Override
  public void periodic() {
  }
}
