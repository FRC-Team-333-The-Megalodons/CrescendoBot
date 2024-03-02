// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TrolleyConstants;

public class Trolley extends SubsystemBase {

  private CANSparkFlex trolleyMotor;
  private DigitalInput limitSwitch;

  private SparkPIDController trolleyController;

  /** Creates a new Trolley. */
  public Trolley() {
    trolleyMotor = new CANSparkFlex(TrolleyConstants.MOTOR_ID, MotorType.kBrushless);
    limitSwitch = new DigitalInput(TrolleyConstants.LIMIT_SWITCH_ID);

    trolleyMotor.restoreFactoryDefaults();

    trolleyController = trolleyMotor.getPIDController();
    trolleyController.setFeedbackDevice(trolleyMotor.getEncoder());
    trolleyController.setP(TrolleyConstants.kP);
    trolleyController.setI(TrolleyConstants.kI);
    trolleyController.setD(TrolleyConstants.kD);
    trolleyController.setFF(TrolleyConstants.kFF);
    trolleyController.setOutputRange(TrolleyConstants.MIN_INPUT, TrolleyConstants.MAX_INPUT);

    trolleyMotor.setIdleMode(IdleMode.kBrake);

    trolleyMotor.burnFlash();
  }

  public void resetEncoder() {
    trolleyMotor.getEncoder().setPosition(0.0);
  }

  public double getPostion() {
    return trolleyMotor.getEncoder().getPosition();
  }

  public void runTrolley(double value) {
    trolleyMotor.set(value);
  }

  public void stopTrolley() {
    trolleyMotor.set(0.0);
  }

  public void setPosition(double setpoint) {
    trolleyController.setReference(setpoint, ControlType.kPosition);

    // TODO: Add some Trolley logic
    // When this function is ran, the trolley will ignore limit switch input and go to its setpoint
    // How will it ignore it? We'll burn that bridge when we get there...
  }

  public boolean atSetpoint(double setpoint) {
    // If our encoder is at a premeditated setpoint, return true, otherwise return false
    if (getPostion() == setpoint) {
      return true;
    } else {
      return false;
    }
  }

  public boolean getLimitSwitch(){
    if (limitSwitch.get() == true) {
      return false;
    }else{
      return true;
    }
  }

  public void zeroPosition(){
    if (getLimitSwitch() == true) {
      //stopTrolley();
      resetEncoder();
    }
  }

  @Override
  public void periodic() {
    zeroPosition();
    SmartDashboard.putNumber("Trolley Pos", getPostion());
    SmartDashboard.putBoolean("LimitSwitch", getLimitSwitch());
    SmartDashboard.putBoolean("At Setpoint?", atSetpoint(getPostion()));
  }
}
