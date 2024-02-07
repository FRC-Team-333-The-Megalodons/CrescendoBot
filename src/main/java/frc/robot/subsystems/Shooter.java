// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private CANSparkFlex topMotor, bottomMotor;

  private PIDController shooterController;

  /** Creates a new Shooter. */
  public Shooter() {
    topMotor = new CANSparkFlex(8, MotorType.kBrushless);
    bottomMotor = new CANSparkFlex(9, MotorType.kBrushless);
    topMotor.setIdleMode(IdleMode.kCoast);
    bottomMotor.setIdleMode(IdleMode.kCoast);
    topMotor.burnFlash();
    bottomMotor.burnFlash();

    bottomMotor.follow(topMotor, true);

    shooterController = new PIDController(0.5, 0, 0);
  }

  public double getVelocity() {
    return topMotor.getEncoder().getVelocity();
  }

  public void runShooter(double value) {
    topMotor.set(value);
  }

  public void stopShooter() {
    topMotor.set(0.0);
  }

  public void setSpeed(double speed) {
    topMotor.set(shooterController.calculate(getVelocity(), speed));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Speed", getVelocity());
    SmartDashboard.putBoolean("At Speed?", shooterController.atSetpoint());
  }
}
