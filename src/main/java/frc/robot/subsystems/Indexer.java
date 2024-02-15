// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Indexer extends SubsystemBase {
    private CANSparkMax indexerMotor;
    public Indexer(){
        indexerMotor = new CANSparkMax(34, MotorType.kBrushless);
        indexerMotor.setIdleMode(IdleMode.kBrake);
    }
    public void index(double value){indexerMotor.set(value);}
}
