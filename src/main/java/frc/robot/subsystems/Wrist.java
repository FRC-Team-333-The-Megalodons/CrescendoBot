// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// one motor
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Wrist extends SubsystemBase {
    private CANSparkMax wristMotor;
   // private DutyCycleEncoder wristEncoder;
   // private PIDController wristPIDController;


    public Wrist() {
        wristMotor = new CANSparkMax(4, MotorType.kBrushless);
        wristMotor.setIdleMode(IdleMode.kBrake);

       // wristPIDController = new PIDController(1.5, 0, 0);
       // wristPIDController.enableContinuousInput(0, 1);

    }
    public void wrist(double value) {wristMotor.set(value);}
    public void wristSTOP(){wristMotor.set(0);}

   /*  public boolean atIntakePositionWrist() {
        if (wristEncoder.getAbsolutePosition() <= 0 && wristEncoder.getAbsolutePosition() >= 0 ) { // intake encoder position 
          return true;
        } else {
            return false;
        }
      }

    public boolean atHomePositionWrist() {
        if (wristEncoder.getAbsolutePosition() <= 0 && wristEncoder.getAbsolutePosition() >= 0) {
            return true;
        } else {
            return false;
        }
    }*/

    @Override
    public void periodic(){
       // SmartDashboard.getNumber("encoderWrist" , wristEncoder.get());
       // SmartDashboard.getBoolean("WristAtIntakePosition", atIntakePositionWrist());
       // SmartDashboard.getBoolean("WristAtHomePosition", atHomePositionWrist());
    }
}
