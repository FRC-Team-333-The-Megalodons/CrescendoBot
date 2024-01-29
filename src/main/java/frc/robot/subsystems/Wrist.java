// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// one motor
package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Wrist extends SubsystemBase {
    private CANSparkFlex wristMotor;
    private DutyCycleEncoder wristEncoder;
    private PIDController wristPIDController;
    

    public Wrist() {
        wristMotor = new CANSparkFlex(0, MotorType.kBrushless);
        wristMotor.setIdleMode(IdleMode.kBrake);

        wristEncoder = new DutyCycleEncoder(0);
        wristEncoder.setConnectedFrequencyThreshold(900);
        wristEncoder.reset();

        wristPIDController = new PIDController(1.5, 0, 0);
        wristPIDController.enableContinuousInput(0, 1);

        

    }
}
