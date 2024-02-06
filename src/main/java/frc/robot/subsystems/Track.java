// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// one motor
package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Track extends SubsystemBase {
    private CANSparkMax trackMotor;
    private DutyCycleEncoder trackEncoder;
    public Track() {
        trackMotor = new CANSparkMax(5, MotorType.kBrushless);
        trackEncoder = new DutyCycleEncoder(7);
        trackEncoder.setConnectedFrequencyThreshold(900);
        trackEncoder.reset();
    }
    public void trackDown(){trackMotor.set(0.2);}
    public void trackUp(){trackMotor.set(-0.2);}
    public void trackStop(){trackMotor.set(0);}

    @Override
    public void periodic(){
        SmartDashboard.getNumber("TrackEncoder", trackEncoder.get());
    }
}
