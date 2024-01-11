package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
      }
      public static final class DriveConstants{
        public static final double  kpTurning = 0.08;
        public static final  double kiTurning = 0;
        public static final double kdTurning = 0;
    
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 6.12/1;
        public static final double kTurningMotorGeraRatio = 150/7/1;
        public static final double kDriveEncoderRot2Meters = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEnocderRot2Rad = kTurningMotorGeraRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meters / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEnocderRot2Rad / 60;
      }
    
}
