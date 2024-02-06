// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Pivot extends SubsystemBase {
    CANSparkFlex pivotMotorRight;
    CANSparkFlex pivotMotorLeft;
    DutyCycleEncoder rightEncoder;
    DutyCycleEncoder leftEncoder;
    PIDController rightPivotPidController;

    public Pivot(){
        pivotMotorRight = new CANSparkFlex(2, MotorType.kBrushless);
        pivotMotorRight.setIdleMode(IdleMode.kBrake);
        pivotMotorLeft = new CANSparkFlex(3, MotorType.kBrushless);
        pivotMotorLeft.setIdleMode(IdleMode.kBrake);

        rightEncoder = new DutyCycleEncoder(0);
        rightEncoder.setConnectedFrequencyThreshold(900);
        rightEncoder.reset();

        


    }
}
