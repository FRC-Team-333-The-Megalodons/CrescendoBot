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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Wrist extends SubsystemBase {
    private CANSparkFlex wristMotor;
    private DutyCycleEncoder wristEncoder;
    private PIDController wristPIDController;
    

    public Wrist() {
        wristMotor = new CANSparkFlex(4, MotorType.kBrushless);
        wristMotor.setIdleMode(IdleMode.kBrake);

        wristPIDController = new PIDController(1.5, 0, 0);
        wristPIDController.enableContinuousInput(0, 1);

        wristEncoder = new DutyCycleEncoder(6);
        wristEncoder.setConnectedFrequencyThreshold(900);
        wristEncoder.reset();
    }
    public void wristUP() {wristMotor.set(0.2);}
    public void wristDOWN() {wristMotor.set(-0.2);}
    public void wristSTOP(){wristMotor.set(0);}

    @Override
    public void periodic(){
        SmartDashboard.getNumber("encoderWrist" , wristEncoder.get());
    }
}
