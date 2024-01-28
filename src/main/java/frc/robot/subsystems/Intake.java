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
  private DutyCycleEncoder intakeEncoder;
  private PIDController intakePIDController;
  private DigitalInput photoElectric_1;
  private DigitalInput photoElectric_2;

  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = new CANSparkFlex(1, MotorType.kBrushless);
    intakeMotor.setIdleMode(IdleMode.kCoast);

    intakeEncoder = new DutyCycleEncoder(0);
    intakeEncoder.setConnectedFrequencyThreshold(900);
    intakeEncoder.reset();

    intakePIDController = new PIDController(1.5, 0, 0);
    intakePIDController.enableContinuousInput(0, 1);

    photoElectric_1 = new DigitalInput(8);
    photoElectric_2 = new DigitalInput(9);
  }

  public void intake() { 
    intakeMotor.set(-0.15); 
  } // intake speed
  
  public void eject() { 
    intakeMotor.set(0.15); 
  } // eject speed 

  public void stopIntake() { 
    intakeMotor.set(0); 
  }

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

  public void resetIntakeEncoder() { 
    intakeMotor.getEncoder().setPosition(0); 
  }
  
  public boolean detect() {
    if (photoElectric_1.get() || photoElectric_2.get()) {
      return true;
    } else { 
      return false;  
    }
}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake encoder", intakeEncoder.get());
    SmartDashboard.getBoolean("NodeIN?", detect());
    SmartDashboard.getBoolean("IsIntakeAutoDone?", intakeAutoDone());
  }
}