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

/** Add your docs here. */
public class Pivot extends SubsystemBase {
    CANSparkFlex pivotMotorRight;
    CANSparkFlex pivotMotorLeft;
    DutyCycleEncoder pivotEncoder;
    PIDController PivotPidController;

    public Pivot(){
       pivotMotorRight = new CANSparkFlex(6, MotorType.kBrushless);
       pivotMotorRight.setIdleMode(IdleMode.kBrake);
       pivotMotorLeft = new CANSparkFlex(7, MotorType.kBrushless);
       pivotMotorLeft.setIdleMode(IdleMode.kBrake);

       PivotPidController = new PIDController(1.5, 0, 0);
       PivotPidController.enableContinuousInput(0, 1);

       pivotEncoder = new DutyCycleEncoder(5);
       pivotEncoder.setConnectedFrequencyThreshold(900);
       pivotEncoder.reset();
    }
    public void pivotUp(){pivotMotorRight.set(0.2); pivotMotorLeft.set(-0.2);}
    public void pivotDown(){pivotMotorRight.set(-0.2); pivotMotorLeft.set(0.2);}
    public void pivotStop(){pivotMotorLeft.set(0); pivotMotorRight.set(0);}

    @Override
    public void periodic(){
        SmartDashboard.getNumber("encoderPivot", pivotEncoder.get());
    }
}
