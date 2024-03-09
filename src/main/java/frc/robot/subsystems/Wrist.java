// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// one motor
package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Trolley;
import frc.robot.Constants.TrolleyConstants;
import frc.robot.Constants.WristConstants;
/** Add your docs here. */
public class Wrist extends SubsystemBase {
    private CANSparkMax wristMotor;
    private PIDController wristPIDController;
    private AbsoluteEncoder wristEncoder;
    private final Trolley m_Trolley = new Trolley();

    public Wrist() {
        wristMotor = new CANSparkMax(WristConstants.WRIST_MOTOR_ID, MotorType.kBrushless);
        wristMotor.setIdleMode(IdleMode.kBrake);
        wristPIDController = new PIDController(WristConstants.kP, WristConstants.kI, WristConstants.kD);
        wristEncoder = wristMotor.getAbsoluteEncoder(com.revrobotics.SparkAbsoluteEncoder.Type.kDutyCycle);
        wristPIDController.enableContinuousInput(WristConstants.MIN_INPUT, WristConstants.MAX_INPUT);
    }
    public void wrist(double value) {
        wristMotor.set(value);
    }
    public void wristSTOP() {
        wristMotor.set(0);
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
    
    public double getPosition() {
        return wristEncoder.getPosition();
      }

    public boolean isWristAtMaxUp() {
    return getPosition() >= WristConstants.MAX_INPUT;
  }

  // public  boolean isWristAtMaxDown() { 
  //   // This considers the elevator state.
  //   if (m_Trolley.trolleyEncoder.getAbsolutePosition() >= TrolleyConstants.PIVOT_POS_LOWEST_POINT_WRIST_CAN_MOVE) {
  //   return getPosition() <= WristConstants.WRIST_POS_LOWER_LIMIT_WHILE_ELEVATOR_UP;
  //   }

  //   if (m_Trolley.trolleyEncoder.getAbsolutePosition() >= TrolleyConstants.ELEVATOR_POS_LOWEST_POINT_ELEVATOR_CAN_GO_WHILE_WRIST_DOWN &&
  //       m_Trolley.trolleyEncoder.getAbsolutePosition() <= TrolleyConstants.ELEVATOR_POS_LOWEST_POINT_WRIST_CAN_MOVE)
  //   {
  //     return getPosition() <= WristConstants.WRIST_POS_LOWER_LIMIT_WHILE_ELEVATOR_DOWN;
  //   }
  //   return false;
  // }


    // PID CONTROLLER (ENCODER POSITION SET IN THE COMMANDS OR ROBOT CONTAINER)
     public void wristPIDController(double position ){
        wristMotor.set(wristPIDController.calculate(getPosition(),position));
    }
    public boolean isDone(){
        wrist
    }

    // SMART DASHBOARD
    @Override
    public void periodic(){
        SmartDashboard.putNumber("encoderWrist" , wristEncoder.getPosition());
        SmartDashboard.putBoolean("WristAtIntakePosition", atIntakePositionWrist());
        SmartDashboard.putBoolean("WristAtHomePosition", atHomePositionWrist());
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
