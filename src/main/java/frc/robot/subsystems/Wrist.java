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
import frc.robot.Constants;
/** Add your docs here. */
public class Wrist extends SubsystemBase {
    private CANSparkMax wristMotor;
    private PIDController wristPIDController;
    private AbsoluteEncoder wristEncoder;


    public Wrist() {
        wristMotor = new CANSparkMax(Constants.WristConstants.WRIST_MOTOR_ID, MotorType.kBrushless);
        wristMotor.setIdleMode(IdleMode.kBrake);
        wristPIDController = new PIDController(Constants.WristConstants.kP, Constants.WristConstants.kI, Constants.WristConstants.kD);
        wristPIDController.enableContinuousInput(Constants.WristConstants.MIN_INPUT, Constants.WristConstants.MAX_INPUT);
        wristEncoder = wristMotor.getAbsoluteEncoder(com.revrobotics.SparkAbsoluteEncoder.Type.kDutyCycle);
    }
    public void wrist(double value) {
        wristMotor.set(value);
    }
    public void wristSTOP() {
        wristMotor.set(0);
    }

       public boolean atIntakePositionWrist() {
        if (wristEncoder.getPosition() == Constants.WristConstants.INTAKE_SETPOINT) { 
          return true;
        } else {
            return false;
        }
      }

    public boolean atHomePositionWrist() {
        if (wristEncoder.getPosition() == Constants.WristConstants.HOME_SETPOINT) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("encoderWrist" , wristEncoder.getPosition());
        SmartDashboard.putBoolean("WristAtIntakePosition", atIntakePositionWrist());
        SmartDashboard.putBoolean("WristAtHomePosition", atHomePositionWrist());
    }
}
