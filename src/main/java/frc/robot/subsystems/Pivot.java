// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {

  private CANSparkFlex pivotMotor1, pivotMotor2;
  private DutyCycleEncoder pivotEncoder;

  private PIDController pivotController;

  /** Creates a new Pivot. */
  public Pivot() {
    pivotMotor1 = new CANSparkFlex(6, MotorType.kBrushless);
    pivotMotor2 = new CANSparkFlex(7, MotorType.kBrushless);
    pivotMotor1.setIdleMode(IdleMode.kBrake);
    pivotMotor2.setIdleMode(IdleMode.kBrake);
    pivotMotor1.burnFlash();
    pivotMotor2.burnFlash();

    //pivotMotor2.follow(pivotMotor1, true);

    pivotEncoder = new DutyCycleEncoder(5);

    pivotController = new PIDController(0.5, 0, 0);
  }

  public double getPosition() {
    return pivotEncoder.getAbsolutePosition();
  }

  public void runPivot(double value) {
    pivotMotor1.set(value);
    pivotMotor2.set(value);
  }

  public void stopPivot() {
    pivotMotor1.set(0.0);
    pivotMotor2.set(0.0);
  }

  public void pivotToSetpoint(double setpoint) {
    pivotMotor1.set(pivotController.calculate(getPosition(), setpoint));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pivot Pos", getPosition());
    SmartDashboard.putBoolean("Pivot Setpoint?", pivotController.atSetpoint());
  }
}
