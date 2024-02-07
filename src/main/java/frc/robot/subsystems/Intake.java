// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  
  private CANSparkFlex intakeMotor;
  private DigitalInput leftSensor;
  private DigitalInput rightSensor;

  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = new CANSparkFlex(3, MotorType.kBrushless);
    intakeMotor.setIdleMode(IdleMode.kCoast);
    intakeMotor.burnFlash();

    leftSensor = new DigitalInput(2);
    rightSensor = new DigitalInput(3);
  }

  public void runIntake(double value) {
    intakeMotor.set(value);
  }

  public void stopIntake() {
    intakeMotor.set(0.0);
  }

  public boolean getNote() {
    if (leftSensor.get() || rightSensor.get()) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Note?", getNote());
  }
}
