// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  CANSparkMax mWrist;
  AbsoluteEncoder mWristAbsoluteEncoder;
  RelativeEncoder mWristEncoder;
  SparkPIDController mWristController;
  public Wrist(int id,double kP, double kI, double kD) {
    mWrist = new CANSparkMax(id, MotorType.kBrushless);
    mWristEncoder = mWrist.getEncoder();
    mWristAbsoluteEncoder = mWrist.getAbsoluteEncoder(Type.kDutyCycle);
    mWristController = mWrist.getPIDController();
    //TODO:Make absloute encoder pass data to reltaive so we could use relative during matches.
    //Absolute Encoder needs to be used only once when the robot is on.
    mWristController.setFeedbackDevice(mWristAbsoluteEncoder);
    mWristController.setP(kP);
    mWristController.setI(kI);
    mWristController.setD(kD);
    mWristController.setOutputRange(-0.5, 0.5);
    mWrist.setIdleMode(IdleMode.kBrake);
    mWrist.burnFlash();
  }
  public void wristGoTo(double setpoint){
    mWristController.setReference(setpoint, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
