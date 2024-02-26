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
    private CANSparkFlex intakeMotor;
    public test(){
        intakeMotor = new CANSparkFlex(Constants.IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
        intakeMotor.setIdleMode(IdleMode.kCoast);

        
    }
}