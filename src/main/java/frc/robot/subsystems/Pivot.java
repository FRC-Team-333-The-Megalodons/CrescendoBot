// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.TrolleyConstants;

public class Pivot extends SubsystemBase {

  private CANSparkFlex pivotMotorLeader, pivotMotorFollower;
  private DutyCycleEncoder pivotEncoder;
  private PhotonCamera camera;

  private PIDController pivotController;

  private final double cameraHeightMeters = Units.inchesToMeters(0);
  private final double targetHeightMeters = Units.inchesToMeters(0);
  private final double cameraPitchRadians = Units.degreesToRadians(0);

  // How far from the target we want to be
  private final double goalRangeMeters = Units.feetToMeters(0);

  private Trolley trolleyRef; 
  /** Creates a new Pivot. */
  public Pivot(Trolley _trolleyRef) {
    trolleyRef = _trolleyRef;
    pivotMotorLeader = new CANSparkFlex(PivotConstants.MOTOR1_ID, MotorType.kBrushless);
    pivotMotorFollower = new CANSparkFlex(PivotConstants.MOTOR2_ID, MotorType.kBrushless);

    pivotMotorLeader.restoreFactoryDefaults();
    pivotMotorFollower.restoreFactoryDefaults();

    pivotMotorLeader.setIdleMode(IdleMode.kBrake);
    pivotMotorFollower.setIdleMode(IdleMode.kBrake);

    pivotMotorLeader.burnFlash();
    pivotMotorFollower.burnFlash();

    pivotMotorFollower.follow(pivotMotorLeader);

    pivotEncoder = new DutyCycleEncoder(PivotConstants.ENCODER_ID);
    // pivotEncoder.setPositionOffset(PivotConstants.ZERO_OFFSET);
    pivotEncoder.setDutyCycleRange(1.0/1024.0, 1023.0/1024.0);

    pivotController = new PIDController(PivotConstants.kP, PivotConstants.kI, PivotConstants.kD);
    // pivotController.enableContinuousInput(0, 1);

    // camera = new PhotonCamera("camera");
  }

  public double getPosition() {
    return pivotEncoder.getAbsolutePosition();
  }

  public void runPivot(double speed) {
    if (mustStopDueToLimit(speed)) {
      stopPivot();
      return;
    }

    pivotMotorLeader.set(speed);
  }

  public void stopPivot() {
    pivotMotorLeader.set(0.0);
  }

  public void runPivotToTargetAngle(double angle) {
    double speed = pivotController.calculate(getPosition(), angle);
    runPivot(speed);
  }

  public void trackTarget() {
    var result = camera.getLatestResult();

    if (result.hasTargets()) {
      pivotMotorLeader.set(pivotController.calculate(result.getBestTarget().getYaw(), 0));
    } else {
      stopPivot();
    }
  }

  private boolean mustStopDueToLimit(double speed)
  {
    return false;
    // // TODO: Is positive Up or Down? This code assumes value > 0 means "go Up", might need to be flipped if not so.
    // return ((speed > 0 && getPosition() >= getUpLimitFromState()) ||
    //         (speed < 0 && getPosition() <= getDownLimitFromState()));
  }

  // Note: DOWN means the shooter is down, and the Intake is up.
  private double getDownLimitFromState()
  {
    if (trolleyRef.getPosition() > TrolleyConstants.INTAKE_SETPOINT_POS) {
      // This intends to say "if the trolley position is towards the front, don't let us move the Pivot down"
      return PivotConstants.HOME_SETPOINT_POS; 
    }
    return PivotConstants.AMP_SETPOINT_POS;
  }

  // Note: UP means the shooter is up, and the Intake is down.
  private double getUpLimitFromState()
  {
    if (trolleyRef.getPosition() < TrolleyConstants.HOME_SETPOINT_POS) {
      // This intends to say "if the trolley position is towards the back, don't let us move the Pivot up"
      return PivotConstants.HOME_SETPOINT_POS;
    }

    return PivotConstants.SHOOTING_SETPOINT_POS;
  }

  @Override
  public void periodic() {
    final String PREFIX = "Pivot ";
    SmartDashboard.putNumber(PREFIX+"Position", getPosition());
    SmartDashboard.putBoolean(PREFIX+"Setpoint", pivotController.atSetpoint());
    SmartDashboard.putNumber(PREFIX+"Up Limit", getUpLimitFromState());
    SmartDashboard.putNumber(PREFIX+"Down Limit", getDownLimitFromState());
  }
}
