// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// one motor
package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TrolleyConstants;
import frc.robot.Constants.WristConstants;
/** Add your docs here. */
public class Wrist extends SubsystemBase {
    private CANSparkMax wristMotor;
    private SparkPIDController wristPIDController;
    private RelativeEncoder wristEncoder;
    private Trolley m_trolleyRef;
    private Pivot m_pivotRef;

    public Wrist() {
        wristMotor = new CANSparkMax(WristConstants.WRIST_MOTOR_ID, MotorType.kBrushless);
        wristMotor.setIdleMode(IdleMode.kBrake);
        wristPIDController = wristMotor.getPIDController();
        wristEncoder = wristMotor.getEncoder();
        wristPIDController.setP(0.05);
        wristPIDController.setI(0);
        wristPIDController.setD(0);
        wristPIDController.setFeedbackDevice(wristEncoder);
    }

    public void setTrollyRef(Trolley trolleyRef)
    {
        m_trolleyRef = trolleyRef;
    }
    public void setPivotRef(Pivot pivotRef)
    {
        m_pivotRef = pivotRef;
    }
    public void wrist(double value) {
        wristMotor.set(value);
    }
    public void wristSTOP() {
        wristMotor.set(0);
    }
    public void wristController(double setpoint){
        wristPIDController.setReference(setpoint, ControlType.kPosition);
    }
    public double getPosition(){return wristEncoder.getPosition();}
    public boolean atSetpoint(){
        if(wristEncoder.getPosition() == 0.122){
            return true;
        } else { return false;}
    }
    public boolean atSetpoint(double min, double max){
        return min<=wristEncoder.getPosition() && max>=wristEncoder.getPosition();
    }
    public void zero(){
        wristEncoder.setPosition(0.0);
    }

    // SMART DASHBOARD 
       public boolean atIntakePositionWrist() {
        if (wristEncoder.getPosition() == WristConstants.NEW_INTAKE_POS) { 
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

    // SMART DASHBOARD
    @Override
    public void periodic(){
        SmartDashboard.putNumber("encoderWrist" , wristEncoder.getPosition());
        SmartDashboard.putBoolean("WristAtIntakePosition", atIntakePositionWrist());
        SmartDashboard.putBoolean("WristAtHomePosition", atHomePositionWrist());
    }

    public boolean isWristAtMaxDown() { 
        // This considers the elevator state.
        /*
        if (m_trolleyRef.trolleyEncoder.getAbsolutePosition() >= TrolleyConstants.PIVOT_POS_LOWEST_POINT_WRIST_CAN_MOVE) {
        return getPosition() <= WristConstants.WRIST_POS_LOWER_LIMIT_WHILE_ELEVATOR_UP;
        }

        if (m_trolleyRef.trolleyEncoder.getAbsolutePosition() >= TrolleyConstants.ELEVATOR_POS_LOWEST_POINT_ELEVATOR_CAN_GO_WHILE_WRIST_DOWN &&
            m_trolleyRef.trolleyEncoder.getAbsolutePosition() <= TrolleyConstants.ELEVATOR_POS_LOWEST_POINT_WRIST_CAN_MOVE)
        {
        return getPosition() <= WristConstants.WRIST_POS_LOWER_LIMIT_WHILE_ELEVATOR_DOWN;
        }
        */
        return false;
    }
}

// FOR THE SMARTDASHBOARD TEST
//    public boolean atShooringPositonWrist() {
    //     if (wristEncoder.getPosition() == WristConstants.NEW_SHOOTING_POS) { 
    //       return true;
    //     } else {
    //         return false;
    //     }
    //   }


    // public boolean atHomePositionWrist() {
    //     if (wristEncoder.getPosition() == WristConstants.HOME_SETPOINT) {
    //         return true;
    //     } else {
    //         return false;
    //     }
    // }

    // public boolean atFirePosition() {
    //     if (wristEncoder.getPosition() == WristConstants.SHOOTING_SETPOINT) {
    //         return true;
    //     } else {
    //         return false;
    //     }
    // }

    // public boolean atAMPPosition() {
    //     if (wristEncoder.getPosition() == WristConstants.AMP_SETPOINT) {
    //         return true;
    //     } else {
    //         return false;
    //     }
    //}
