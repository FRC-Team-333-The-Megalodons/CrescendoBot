// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {

  private CANSparkFlex pivotMotor1, pivotMotor2;
  private DutyCycleEncoder pivotEncoder;
  private PhotonCamera camera;

  private PIDController pivotController;

  private final double cameraHeightMeters = Units.inchesToMeters(0);
  private final double targetHeightMeters = Units.inchesToMeters(0);
  private final double cameraPitchRadians = Units.degreesToRadians(0);

  // How far from the target we want to be
  private final double goalRangeMeters = Units.feetToMeters(0);

  /** Creates a new Pivot. */
  public Pivot() {
    pivotMotor1 = new CANSparkFlex(6, MotorType.kBrushless);
    pivotMotor2 = new CANSparkFlex(7, MotorType.kBrushless);

    pivotMotor1.restoreFactoryDefaults();
    pivotMotor2.restoreFactoryDefaults();

    pivotMotor1.setIdleMode(IdleMode.kBrake);
    pivotMotor2.setIdleMode(IdleMode.kBrake);

    pivotMotor1.burnFlash();
    pivotMotor2.burnFlash();

    //pivotMotor2.follow(pivotMotor1, true);

    pivotEncoder = new DutyCycleEncoder(9);
    pivotEncoder.setDistancePerRotation(PivotConstants.PIVOT_CONVERSION_FACTOR);

    pivotController = new PIDController(0.5, 0, 0);

    // camera = new PhotonCamera("camera");
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

  public void setAngle(double angle) {
    pivotMotor1.set(pivotController.calculate(getPosition(), angle));
  }

  public void trackTarget() {
    var result = camera.getLatestResult();

    if (result.hasTargets()) {
      pivotMotor1.set(pivotController.calculate(result.getBestTarget().getYaw(), 0));
    } else {
      stopPivot();
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pivot Pos", getPosition());
    SmartDashboard.putBoolean("Pivot Setpoint?", pivotController.atSetpoint());
  }
}
