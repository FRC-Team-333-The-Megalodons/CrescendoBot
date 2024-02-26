// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// two motors 
package frc.robot.subsystems;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;;

/** Add your docs here. */
public class Shooter extends SubsystemBase{
    private CANSparkFlex shooterTop;
    private CANSparkFlex shooterBottom;
    private SparkPIDController FirePIDController;
    
    public Shooter() {
        shooterTop = new CANSparkFlex(ShooterConstants.FIRE_TOP_MOTOR_ID, MotorType.kBrushless);
        shooterTop.setIdleMode(IdleMode.kCoast);
        shooterBottom = new CANSparkFlex(ShooterConstants.FIRE_BOTTOM_MOTOR_ID, MotorType.kBrushless);
        shooterBottom.setIdleMode(IdleMode.kCoast);

        shooterTop.restoreFactoryDefaults();
        FirePIDController = shooterTop.getPIDController();
        FirePIDController.setFeedbackDevice(shooterTop.getEncoder());
        FirePIDController.setP(ShooterConstants.kP);
        FirePIDController.setI(ShooterConstants.kI);
        FirePIDController.setD(ShooterConstants.kD);
        FirePIDController.setOutputRange(ShooterConstants.MIN_INPUT, ShooterConstants.MAX_INPUT);
        shooterTop.burnFlash();
    }
    public void idleFire(double value) {
        shooterTop.set(value);
        shooterBottom.follow(shooterTop);
    }
    public void fire(double value) {
        shooterTop.set(value);
        shooterBottom.follow(shooterTop);
    }
    public void fireStop() {
        shooterTop.set(0);
        shooterBottom.follow(shooterTop);
    }

    public double getVelocity() {
        return shooterTop.getEncoder().getVelocity();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("VelocityShoot", getVelocity());
    }
}
