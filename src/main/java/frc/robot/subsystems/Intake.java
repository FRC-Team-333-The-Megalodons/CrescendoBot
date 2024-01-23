// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private CANSparkFlex intakeMotor;

  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = new CANSparkFlex(1, MotorType.kBrushless);
    intakeMotor.setIdleMode(IdleMode.kCoast);
  }

  public void intake() { intakeMotor.set(-0.15); } // intake speed
  

  public void outake() { intakeMotor.set(0.15); } // outake speed 

  public void notake() { intakeMotor.set(0); }

  public boolean intakeAutoDone() {
    if (intakeMotor.getEncoder().getPosition() <= -0) { // intake encoder position 
      return true;
    }
    return false;
  }

  public boolean outakeAutoDone() {
    if (intakeMotor.getEncoder().getPosition() >= 0) { // outake encoder positon 
      return true;
    }
    return false;
  }

  public void resetIntakeEncoder() { intakeMotor.getEncoder().setPosition(0); }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake encoer", intakeMotor.getEncoder().getPosition());
  }
}