// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Add your docs here. */
    public class test extends SubsystemBase{
    private CANSparkFlex testMotor;
    public test(){
        testMotor = new CANSparkFlex(Constants.IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
        testMotor.setIdleMode(IdleMode.kCoast);
    }
    public void testIN(){
        testMotor.set(0.2);
    }
    public void testOUT(){
        testMotor.set(-0.2);
    }
    public void testSTOP(){
        testMotor.set(0);
    }
}