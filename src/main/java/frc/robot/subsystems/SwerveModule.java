package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class SwerveModule extends SubsystemBase {
  private final CANcoder absoluteEncoder;
  private final CANSparkFlex driveMotor;
  private final CANSparkFlex turningMotor;
  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder pivotEncoder;
  private final boolean absoluteEncoderReversed;
  private final double absoluteEncoderOffsetRad;
  private final PIDController turningPidController;
  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReverse, 
  boolean turningMotorReverse,int absoluteEncoderId, double absoluteEncoderOffset,boolean absoluteEncoderReversed){
    absoluteEncoder = new CANcoder(absoluteEncoderId);
    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    driveMotor = new CANSparkFlex(absoluteEncoderId, null);
    turningMotor = new CANSparkFlex(absoluteEncoderId, null);
    driveEncoder =  driveMotor.getEncoder();
    pivotEncoder  = turningMotor.getEncoder();
    turningPidController = new PIDController(Constants.DriveConstants.kpTurning,DriveConstants.kiTurning,DriveConstants.kdTurning);
    turningPidController.enableContinuousInput(-Math.PI, Math.PI);

    driveEncoder.setPositionConversionFactor(DriveConstants.kDriveEncoderRot2Meters);
    driveEncoder.setVelocityConversionFactor(DriveConstants.kDriveEncoderRPM2MeterPerSec);
    pivotEncoder.setPositionConversionFactor(DriveConstants.kTurningEnocderRot2Rad);
    pivotEncoder.setVelocityConversionFactor(DriveConstants.kTurningEncoderRPM2RadPerSec);

  }
  public double getDrivePosition(){ return driveEncoder.getPosition();}
  public double getVelocityOfDrivingMotor(){return driveEncoder.getVelocity();}
  public double getVoltageOfDrivingMotor(){return driveMotor.getBusVoltage();}

  public double getTurningPosition(){return pivotEncoder.getPosition();}
  public double getVoltageOfTurningMotor(){return turningMotor.getBusVoltage();}
  public double getVelocityOfTurningMotor(){return pivotEncoder.getVelocity();}

  public double getPositiontInRad(){
    double angle = turningMotor.getVoltageCompensationNominalVoltage()/ turningMotor.getBusVoltage();
    angle*=angle*2*Math.PI;
    return angle*(absoluteEncoderReversed ? -1.0:1.0);
  }

  

  public void invertTurningMotor(){
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    turningPidController.atSetpoint();
  }

}
