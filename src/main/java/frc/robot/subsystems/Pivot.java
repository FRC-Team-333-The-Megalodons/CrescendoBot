// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;

/** Add your docs here. */
public class Pivot extends SubsystemBase {
    CANSparkFlex pivotMotorRight;
    CANSparkFlex pivotMotorLeft;
    DutyCycleEncoder pivotEncoder;
    PIDController PivotPidController;
    private Trolley m_trolleyRef;
    private Wrist m_wristRef;

    public Pivot(){
       pivotMotorRight = new CANSparkFlex(PivotConstants.PIVOT_MOTOR1_ID, MotorType.kBrushless);
       pivotMotorRight.setIdleMode(IdleMode.kBrake);
       pivotMotorLeft = new CANSparkFlex(PivotConstants.PIVOT_MOTOR2_ID, MotorType.kBrushless);
       pivotMotorLeft.setIdleMode(IdleMode.kBrake);
       pivotMotorLeft.follow(pivotMotorRight);
       PivotPidController = new PIDController(PivotConstants.kP, PivotConstants.kI, PivotConstants.kD);
       PivotPidController.enableContinuousInput(PivotConstants.MIN_INPUT, PivotConstants.MAX_INPUT);
    
        pivotEncoder = new DutyCycleEncoder(PivotConstants.PIVOT_ENCODER_ID);
        pivotEncoder.setDistancePerRotation(900);
        pivotEncoder.reset();
    }
    public void setTrolleyRef(Trolley trolleyRef)
    {
        m_trolleyRef = trolleyRef;
    }
    public void setWristRef(Wrist wristRef)
    {
        m_wristRef = wristRef;
    }
    public void pivot(double value) {
        // Negative value means "up". Positive value means "down".
        if (value < 0) {
            if (!isOkToMovePivotUp()) {
                pivotStop();
                return;
            }
        } else if (value > 0) {
            if (!isOkToMovePivotDown()) {
                pivotStop();
                return;
            }
        }
        pivotMotorRight.set(value);
    }
    public void pivotStop() {
        pivotMotorRight.set(0);
    }

    public boolean fuzzyEquals(double a, double b)
    {
        final double epsilon = 0.001;
        return Math.abs(a-b) < epsilon;
    }

     public boolean atIntakePositionPivot() {
        return fuzzyEquals(getPivotPosition(), PivotConstants.INTAKE_SETPOINT);
      }

    public boolean atSpeakerPositionPivot() {
        return fuzzyEquals(getPivotPosition(), PivotConstants.SPEAKER_SERPOINT);
    }

    public boolean atHomePositionPivot() {
        return fuzzyEquals(getPivotPosition(), PivotConstants.HOME_SETPOINT);
    }

    public boolean atAMPPositionPivot() {
        return fuzzyEquals(getPivotPosition(), PivotConstants.AMP_SETPOINT);
    }
    
      public void resetPivotEncoderPivot() { 
        pivotMotorRight.getEncoder().setPosition(0); 
      } 

    public boolean isOkToMovePivotUp()
    {
        // For now, just return true here because my checks aren't working
        return true;
        /*
        if (m_trolleyRef.isTrolleyTooFarInToPivotUpPastBumper())
        {
            return getPivotPosition() > PivotConstants.PIVOT_UP_FAR_ENOUGH_THAT_TROLLEY_COULD_HIT_BACK_BUMPER;
        }
        if (m_trolleyRef.isTrolleyTooFarInToPivotVertical())
        {
            return getPivotPosition() > PivotConstants.PIVOT_UP_FAR_ENOUGH_THAT_TROLLEY_COULD_HIT_UNDERBELLY;
        }
        return true;
        */
    }

    public boolean isOkToMovePivotDown()
    {
        if (m_trolleyRef.isTrolleyOut())
        {
            // If the Trolley is out, then we can only move down if we're above the "trolley can move safely" setpoint.
            return getPivotPosition() > PivotConstants.PIVOT_FURTHEST_DOWN_WHERE_TROLLEY_CAN_MOVE;
        }
        return true;
    }

    public boolean isPivotAtMaxUp()
    {
        // TODO
        return false;
    }

    public boolean isPivotAtMaxDown()
    {
        // TODO
        return false;
    }

    public double getPivotPosition()
    {
        return (pivotEncoder.getAbsolutePosition() * -1) + 1.0;
    }

    //   public double getPosition() {
    //     return pivotEncoder.getAbsolutePosition();
    //   }

    //   public void goIntakePivot() {
    //     pivotMotorRight.set(PivotPidController.calculate(getPosition(), PivotConstants.INTAKE_SETPOINT));
    //   }// if you read this, you have the gay

    //   public void goHomePivot() {
    //     pivotMotorRight.set(PivotPidController.calculate(getPosition(), PivotConstants.HOME_SETPOINT));
    //   }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("encoderPivot", getPivotPosition());
        SmartDashboard.putBoolean("PivotAtIntakePosition?", atIntakePositionPivot());
        SmartDashboard.putBoolean("PivotAtSpeakerPosition?", atSpeakerPositionPivot());
        SmartDashboard.putBoolean("PivotAtHomePosition", atHomePositionPivot());
    }
}

