// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// one motor
package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TrolleyConstants;
import com.revrobotics.SparkPIDController;

/** Add your docs here. */
public class Trolley extends SubsystemBase {
    private CANSparkFlex trolleyMotor;
    private DigitalInput limitSwitch;
    private SparkPIDController trolleyController;
    DutyCycleEncoder trolleyEncoder;
    private Pivot m_pivotRef;
    private Wrist m_wristRef;

    public Trolley() {
        trolleyMotor = new CANSparkFlex(TrolleyConstants.TROLLEY_MOTOR_ID, MotorType.kBrushless);
        limitSwitch = new DigitalInput(TrolleyConstants.TROLLEY_LIMIT_SWITCH_ID);

        trolleyMotor.restoreFactoryDefaults();
        trolleyController = trolleyMotor.getPIDController();
        trolleyController.setFeedbackDevice(trolleyMotor.getEncoder());
        trolleyController.setP(TrolleyConstants.kP);
        trolleyController.setI(TrolleyConstants.kI);
        trolleyController.setD(TrolleyConstants.kD);
        trolleyController.setOutputRange(TrolleyConstants.MIN_INPUT, TrolleyConstants.MAX_INPUT);
        trolleyMotor.setIdleMode(IdleMode.kBrake);
        trolleyMotor.burnFlash();

        trolleyEncoder = new DutyCycleEncoder(TrolleyConstants.TROLLEY_ENCODER_ID);
        trolleyEncoder.setDistancePerRotation(900);
        trolleyEncoder.reset();
    }
    public void setPivotRef(Pivot pivotRef)
    {
        m_pivotRef = pivotRef;
    }
    public void setWristRef(Wrist wristRef)
    {
        m_wristRef = wristRef;
    }
    public void trolley(double value) {
        trolleyMotor.set(value);
    }
    public void trolleyStop() {
        trolleyMotor.set(0);
    }

    
     public boolean atHomePositionTrack() {
        if (trolleyMotor.getEncoder().getPosition() == TrolleyConstants.HOME_SETPOINT) {
            return true;
        } else {
            return false;
        }
    }

    public boolean atIntakePositionTrack() {
        if (trolleyMotor.getEncoder().getPosition() == TrolleyConstants.INTAKE_SETPOINT) {
            return true;
        } else {
            return false;
        }
    }
    public boolean atAMPPosition() {
        if (trolleyMotor.getEncoder().getPosition() == TrolleyConstants.AMP_SETPOINT) {
            return true;
        } else {
            return false;
        }
    }

    
    public boolean getLimitSwitch() {
        if (limitSwitch.get() == true) {
            return false;
          }else{
            return true;
          }
    }

    public boolean isTrolleyAtMaxForward() { 
        // TODO
        return false;
    }
    public boolean isTrolleyAtMaxBack() {
        // TODO
        return false;
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("TrolleyLimitSwitch", getLimitSwitch());
        SmartDashboard.putNumber("encoderWrist" , trolleyEncoder.getAbsolutePosition());
        SmartDashboard.putBoolean("TrackHomePosition", atHomePositionTrack());
        SmartDashboard.putBoolean("TrackIntakePosition", atIntakePositionTrack());
    }
}
