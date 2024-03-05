// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;

import java.util.Date;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TrolleyConstants;

public class Trolley extends SubsystemBase {

  private CANSparkFlex trolleyMotor;
  private DigitalInput limitSwitch;
  private DutyCycleEncoder trolleyEncoder;

  private SparkPIDController trolleyController;

  private Pivot pivotRef; // Needed to check limits.

  /** Creates a new Trolley. */
  public Trolley(Pivot _pivotRef) {
    this();
    this.setPivotRef(_pivotRef);
  }
  
  public Trolley()
  {
    trolleyMotor = new CANSparkFlex(TrolleyConstants.MOTOR_ID, MotorType.kBrushless);
    trolleyEncoder = new DutyCycleEncoder(TrolleyConstants.ENCODER_ID);
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

  public void setPivotRef(Pivot _pivotRef)
  {
    pivotRef = _pivotRef;
  }

  /*
  public void resetEncoder() {
    trolleyMotor.getEncoder().setPosition(0.0);
  }
  */

  public double getPosition() {
    return trolleyEncoder.get();
    // return trolleyEncoder.getAbsolutePosition();
    //return trolleyMotor.getEncoder().getPosition();
  }

  public void runTrolley(double speed) {
    if (mustStopDueToLimit(speed)) {
      stopTrolley();
      return;
    }
  
    trolleyMotor.set(speed);
  }

  public void stopTrolley() {
    trolleyMotor.set(0.0);
  }

  public void setPosition(double setpoint) {
    // We can "guess" at what direction it'll set:
    double direction = (setpoint > getPosition() ? 1.0 : -1.0);
    if (mustStopDueToLimit(direction)) {
      stopTrolley(); // TODO: Verify that doing `set` on the motor cancels the closed-loop mode by setReference (I hope so, nothing in the documentation explains how to do it if not)
      return;
    }
    trolleyController.setReference(setpoint, ControlType.kPosition);
  }

  public boolean atSetpoint(double setpoint) {
    // If our encoder is at a premeditated setpoint, return true, otherwise return false
    return (getPosition() == setpoint);
  }

  public boolean getLimitSwitch(){
    return !limitSwitch.get();
  }

  /*
  public void zeroPosition(){
    if (getLimitSwitch() == true) {
      //stopTrolley();
      resetEncoder();
    }
  }
  */

  private boolean mustStopDueToLimit(double speed)
  {
    return false;
    // // TODO: Is positive Front or Back? This code assumes value > 0 means "go Up", might need to be flipped if not so.
    // return ((speed > 0 && getPosition() >= getFrontLimitFromState()) ||
    //         (speed < 0 && getPosition() <= getBackLimitFromState()));
  }

  private double getBackLimitFromState()
  {
    // TODO
    return TrolleyConstants.DOWN_LIMIT_POS;
  }

  private double getFrontLimitFromState()
  {
    return TrolleyConstants.HOME_SETPOINT_POS;
  }

  @Override
  public void periodic() {
    //zeroPosition();
    SmartDashboard.putNumber("Trolley Pos", getPosition());
    SmartDashboard.putBoolean("LimitSwitch", getLimitSwitch());
    SmartDashboard.putBoolean("At Setpoint?", atSetpoint(getPosition()));
  }
}
