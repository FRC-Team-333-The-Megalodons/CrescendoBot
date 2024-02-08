// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {

  private CANSparkMax wristMotor;
  private AbsoluteEncoder wristEncoder;

  private PIDController wristController;

  /** Creates a new Wrist. */
  public Wrist() {
    wristMotor = new CANSparkMax(4, MotorType.kBrushless);
    wristMotor.setIdleMode(IdleMode.kBrake);
    wristMotor.burnFlash();

    wristEncoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);

    wristController = new PIDController(0.5, 0.0, 0.0);
  }

  public double getPosition() {
    return wristEncoder.getPosition();
  }

  public void runWrist(double value) {
    wristMotor.set(value);
  }

  public void stopWrist() {
    wristMotor.set(0.0);
  }

  public void wristToSetpoint(double setpoint) {
    wristMotor.set(wristController.calculate(getPosition(), setpoint));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Pos", getPosition());
    SmartDashboard.putBoolean("Wrist Setpoint?", wristController.atSetpoint());
  }
}
