// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// one motor
package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private CANSparkFlex intakeMotor;
  private DigitalInput peLeft;
  private DigitalInput peRight;

  private DutyCycleEncoder intakeEncoder;
  private PIDController intakePIDController;

  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = new CANSparkFlex(3, MotorType.kBrushless);
    intakeMotor.setIdleMode(IdleMode.kCoast);

    intakeEncoder = new DutyCycleEncoder(0);
    intakeEncoder.setConnectedFrequencyThreshold(900);
    intakeEncoder.reset();

    intakePIDController = new PIDController(1.5, 0, 0);
    intakePIDController.enableContinuousInput(0, 1);

    peLeft = new DigitalInput(3);
    peRight = new DigitalInput(2);
  }
    public void intake(double value){intakeMotor.set(value);}
    public void intakeStop(){intakeMotor.set(0);}

  public boolean intakeAutoDone() {
    if (intakeMotor.getEncoder().getPosition() <= 0) { // intake encoder position 
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

  public void resetIntakeEncoder() { 
    intakeMotor.getEncoder().setPosition(0); 
  }
  
  public boolean detectNote(){
    if (peLeft.get() || peRight.get()){
        return true;
    } else {return false; }
}

@Override
public void periodic(){
    SmartDashboard.putBoolean("Left", peLeft.get());
    SmartDashboard.putBoolean("Right", peRight.get());
    SmartDashboard.putBoolean("GetNote", detectNote());

    SmartDashboard.putNumber("encoder", intakeEncoder.get());
    SmartDashboard.putBoolean("Encoder at positoin?", intakePIDController.atSetpoint());
  }
}