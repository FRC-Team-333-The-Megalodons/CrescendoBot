// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TrolleyConstants;

public class Trolley extends SubsystemBase {
  /** Creates a new Trolly. */
  CANSparkMax m_trolley;
  RelativeEncoder m_trolleyEncoder;
  AbsoluteEncoder m_trolleyAbsolute;
  DigitalInput home, out;
  SparkPIDController trolleyController;
  public Trolley(int id, int homeID, int outID, double kP, double kI, double kD) {
    m_trolley = new CANSparkMax(id, MotorType.kBrushless);
    home = new DigitalInput(homeID);
    out = new DigitalInput(outID);
    m_trolleyAbsolute = m_trolley.getAbsoluteEncoder(Type.kDutyCycle);
    m_trolleyEncoder = m_trolley.getEncoder();
    trolleyController = m_trolley.getPIDController();
    trolleyController.setP(kP);
    trolleyController.setI(kI);
    trolleyController.setD(kD);
    //TODO:Make absloute encoder pass data to reltaive so we could use relative during matches.
    //Absolute Encoder needs to be used only once when the robot is on.
    trolleyController.setFeedbackDevice(m_trolleyAbsolute);
    trolleyController.setOutputRange(Constants.TrolleyConstants.minPIDoutput, TrolleyConstants.maxPIDoutput);

  }
  public void stopTrolly(){
    m_trolley.set(0.0);
  }
  public void runTrolly(double speed){
    m_trolley.set(speed);
  }
  public void trolleyTo(double setpoint){
    trolleyController.setReference(setpoint, ControlType.kPosition);
  }
  public boolean IN(){
    if (home.get()) {
      return true;
    }else{
      return false;
    }
  }
  public boolean OUT(){
    if (out.get()) {
      return true;
    }else{
      return false;
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("HOME?", IN());
    SmartDashboard.putBoolean("OUT", OUT());
    SmartDashboard.putNumber("TrollyAbs", m_trolleyAbsolute.getPosition());
  }
}
