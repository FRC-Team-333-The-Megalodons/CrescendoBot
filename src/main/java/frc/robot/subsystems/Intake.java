// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private CANSparkFlex m_intake;
  private DigitalInput rightPES, leftPES;
  public Intake(int id, int rightChannel, int leftChannel) {
    m_intake = new CANSparkFlex(id, MotorType.kBrushless);
    rightPES = new DigitalInput(rightChannel);
    leftPES = new DigitalInput(leftChannel);
  }
  public boolean noteIsIN(){
    if (rightPES.get() || leftPES.get()) {
      return true;
    }else{
      return false;
    }
  }
  public void stopIntake(){
    m_intake.set(0.0);  
  }
  public void runIntake(double speed){
    m_intake.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Right PES", rightPES.get());
    SmartDashboard.putBoolean("Left PES", leftPES.get());
  }
}
