// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// one motor
package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private CANSparkFlex intakeMotor;
  private DigitalInput peLeft;
  private DigitalInput peRight;
  

  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = new CANSparkFlex(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    intakeMotor.setIdleMode(IdleMode.kCoast);
    peLeft = new DigitalInput(IntakeConstants.LEFT_SENSOR_ID);
    peRight = new DigitalInput(IntakeConstants.RIGHT_SENSOR_ID);
     
  }
    public void intake(double value) {
      intakeMotor.set(value);
    }
    public void intakeStop() {
      intakeMotor.set(0);
    }
  
  public boolean detectNote(){
    if (peLeft.get() || peRight.get()){
        return true;
    } else {return false; }
}

@Override
public void periodic(){
    SmartDashboard.putBoolean("GetNote", detectNote());
  }
}