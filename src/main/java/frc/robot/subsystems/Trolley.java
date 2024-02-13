// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Trolley extends SubsystemBase {

  private CANSparkMax trolleyMotor;
  private DigitalInput limitSwitch;

  private PIDController trolleyController;

  /** Creates a new Trolley. */
  public Trolley() {
    trolleyMotor = new CANSparkMax(5, MotorType.kBrushless);
    limitSwitch = new DigitalInput(7);
    trolleyMotor.restoreFactoryDefaults();
    trolleyMotor.setIdleMode(IdleMode.kBrake);
    trolleyMotor.burnFlash();

    trolleyController = new PIDController(0.5, 0, 0);
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

  public void trolleyToSetpoint(double setpoint) {
    trolleyMotor.set(trolleyController.calculate(getPostion(), setpoint));
  }

  public boolean getLimitSwitch(){
    if (limitSwitch.get() == true) {
      return false;
    }else{
      return true;
    }
  }

  public void resetPose(){
    if (getLimitSwitch() == true) {
      resetEncoder();
    }
  }

  @Override
  public void periodic() {
    resetPose();
    SmartDashboard.putNumber("Trolley Pos", getPostion());
    SmartDashboard.putBoolean("Tolley Setpoint?", trolleyController.atSetpoint());
    SmartDashboard.putBoolean("LimitSwitch", getLimitSwitch());
  }
}
