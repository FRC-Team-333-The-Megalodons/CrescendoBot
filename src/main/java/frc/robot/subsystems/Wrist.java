// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// one motor
package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
/** Add your docs here. */
public class Wrist extends SubsystemBase {
    private CANSparkMax wristMotor;
    private PIDController wristPIDController;
    private AbsoluteEncoder wristEncoder;


    public Wrist() {
        wristMotor = new CANSparkMax(WristConstants.WRIST_MOTOR_ID, MotorType.kBrushless);
        wristMotor.setIdleMode(IdleMode.kBrake);
        wristPIDController = new PIDController(WristConstants.kP, WristConstants.kI, WristConstants.kD);
        wristPIDController.enableContinuousInput(WristConstants.MIN_INPUT, WristConstants.MAX_INPUT);
        wristEncoder = wristMotor.getAbsoluteEncoder(com.revrobotics.SparkAbsoluteEncoder.Type.kDutyCycle);
        wristPIDController.setOutputRange(WristConstants.MIN_INPUT, WristConstants.MAX_INPUT);
        wristPIDController.setPositionPIDWrappingEnabled(true);
        wristPIDController.setPositionPIDWrappingMinInput(WristConstants.INTAKE_SETPOINT);
        wristPIDController.setPositionPIDWrappingMaxInput(WristConstants.SHOOTING_SETPOINT);
    }
    public void wrist(double value) {
        wristMotor.set(value);
    }
    public void wristSTOP() {
        wristMotor.set(0);
    }

       public boolean atIntakePositionWrist() {
        if (wristEncoder.getPosition() == WristConstants.NEW_INTAKE_POS) { 
          return true;
        } else {
            return false;
        }
      }

       public boolean atShooringPositonWrist() {
        if (wristEncoder.getPosition() == WristConstants.NEW_SHOOTING_POS) { 
          return true;
        } else {
            return false;
        }
      }


    public boolean atHomePositionWrist() {
        if (wristEncoder.getPosition() == WristConstants.HOME_SETPOINT) {
            return true;
        } else {
            return false;
        }
    }

    public boolean atFirePosition() {
        if (wristEncoder.getPosition() == WristConstants.SHOOTING_SETPOINT) {
            return true;
        } else {
            return false;
        }
    }

    public boolean atAMPPosition() {
        if (wristEncoder.getPosition() == WristConstants.AMP_SETPOINT) {
            return true;
        } else {
            return false;
        }
    }

    public void setPosition(double setpoint) {
        wristPIDController.setReference(setpoint, ControlType.kPosition);
      }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("encoderWrist" , wristEncoder.getPosition());
        SmartDashboard.putBoolean("WristAtIntakePosition", atIntakePositionWrist());
        SmartDashboard.putBoolean("WristAtHomePosition", atHomePositionWrist());
    }
}
