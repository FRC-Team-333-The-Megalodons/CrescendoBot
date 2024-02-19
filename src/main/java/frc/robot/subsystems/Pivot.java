// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

/** Add your docs here. */
public class Pivot extends SubsystemBase {
    CANSparkFlex pivotMotorRight;
    CANSparkFlex pivotMotorLeft;
    DutyCycleEncoder pivotEncoder;
    PIDController PivotPidController;

    public Pivot(){
       pivotMotorRight = new CANSparkFlex(PivotConstants.PIVOT_MOTOR1_ID, MotorType.kBrushless);
       pivotMotorRight.setIdleMode(IdleMode.kBrake);
       pivotMotorLeft = new CANSparkFlex(PivotConstants.PIVOT_MOTOR2_ID, MotorType.kBrushless);
       pivotMotorLeft.setIdleMode(IdleMode.kBrake);
       PivotPidController = new PIDController(PivotConstants.kP, PivotConstants.kI, PivotConstants.kD);
       PivotPidController.enableContinuousInput(PivotConstants.MIN_INPUT, PivotConstants.MAX_INPUT);
    
    pivotEncoder = new DutyCycleEncoder(PivotConstants.PIVOT_ENCODER_ID);
    pivotEncoder.setDistancePerRotation(900);
    }
    public void pivot(double value) {
        pivotMotorRight.set(value);
        pivotMotorLeft.follow(pivotMotorRight);
    }
    public void pivotStop() {
        pivotMotorRight.set(0);
        pivotMotorLeft.follow(pivotMotorRight);
    }

     public boolean atIntakePositionPivot() {
        if (pivotEncoder.getAbsolutePosition() == PivotConstants.INTAKE_SETPOINT) { // intake encoder position 
          return true;
        } else {
            return false;
        }
      }

    public boolean atSpeakerPositionPivot() {
        if (pivotEncoder.getAbsolutePosition() == PivotConstants.SPEAKER_SERPOINT) {
            return true;
        } else {
            return false;
        }
    }

    public boolean atHomePositionPivot() {
        if (pivotEncoder.getAbsolutePosition() == PivotConstants.HOME_SETPOINT) {
            return true;
        } else {
            return false;
        }
    }

    public boolean atAMPPositionPivot() {
        if (pivotEncoder.getAbsolutePosition() == PivotConstants.AMP_SETPOINT) {
            return true;
        } else {
            return false;
        }
    }
    
      public void resetPivotEncoderPivot() { 
        pivotMotorRight.getEncoder().setPosition(0); 
      } 

    @Override
    public void periodic(){
        SmartDashboard.putNumber("encoderPivot", pivotEncoder.getAbsolutePosition());
        SmartDashboard.putBoolean("PivotAtIntakePosition?", atIntakePositionPivot());
        SmartDashboard.putBoolean("PivotAtSpeakerPosition?", atSpeakerPositionPivot());
        SmartDashboard.putBoolean("PivotAtHomePosition", atHomePositionPivot());
    }
}

