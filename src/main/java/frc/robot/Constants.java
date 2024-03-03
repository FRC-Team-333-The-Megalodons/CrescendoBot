// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {
    public static final class IntakeConstants {
        public static final int INTAKE_MOTOR_ID = 3;
        public static final int RIGHT_SENSOR_ID = 1;
        public final static int LEFT_SENSOR_ID = 0;
    }
    public static final class WristConstants {
        public static final int WRIST_MOTOR_ID = 4;
        public static final double HOME_SETPOINT = 0.0;
        public static final double INTAKE_SETPOINT = 0.374;
        public static final double SHOOTING_SETPOINT = 0.164;
        public static final double AMP_SETPOINT = 0.488;
        public static final double kP = 0.3;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double MIN_INPUT = -1.0;
        public static final double MAX_INPUT = 1.0;

        public static final double NEW_INTAKE_POS = 0.664; //6897
        public static final double NEW_SHOOTING_POS = 0.3267;
        public static final double WRIST_POS_LOWER_LIMIT_WHILE_ELEVATOR_UP = 0;
        public static final double WRIST_POS_LOWER_LIMIT_WHILE_ELEVATOR_DOWN = 0;

        /*
         * wrist intake position 0.6897
         * swrist shooting position 0.3267
         */
    }
    public static final class TrolleyConstants {
      public static final int TROLLEY_MOTOR_ID = 5;
      public static final int TROLLEY_LIMIT_SWITCH_ID = 7;
      public static final int TROLLEY_ENCODER_ID = 3;
      public static final double HOME_SETPOINT = 0;
      public static final double INTAKE_SETPOINT = 85;
      public static final double AMP_SETPOINT = 69;
      public static final double kP = 0.2;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double MIN_INPUT = -1.0;
      public static final double MAX_INPUT = 1.0;
      public static final double WRIST_POS_LOWER_LIMIT_WHILE_TROLLEY_DOWN = 0.7024;
      public static final double TROLLEY_POS_LOWEST_POINT_WRIST_CAN_MOVE = 4.185;                                                 
    }
    public static final class PivotConstants {
    public static final int PIVOT_MOTOR1_ID = 6;
    public static final int PIVOT_MOTOR2_ID = 7; 
    public static final int PIVOT_ENCODER_ID = 9; 
    public static final double HOME_SETPOINT = 0.0;
    public static final double INTAKE_SETPOINT = 0.0;
    public static final double SPEAKER_SERPOINT = 0.0;
    public static final double AMP_SETPOINT = 0.0;
    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double MIN_INPUT = -1.0;
    public static final double MAX_INPUT = 1.0;
          public static final double PIVOT_POS_LOWEST_POINT_WRIST_CAN_MOVE = 0;
    }
    public static final class ShooterConstants {
        public static final int FIRE_TOP_MOTOR_ID = 8;
        public static final int FIRE_BOTTOM_MOTOR_ID = 9;
        public static final int INDEX_MOTOR_ID = 15;
        public static final double SHOT_RPM = 6000;
        public static final double IDLE_RPM = 1000;
        public static final double kP = 0.0004;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kFF = 0.0;
        public static final double MIN_INPUT = -1;
        public static final double MAX_INPUT = 1;
    }
    public static final class IndexerConstants {
        public static final int INDEXER_MOTOR_ID = 15;
    }
    public static final class LEDConstants {
        public static final int LED_PORT_ID = 9;
        public static final int NUMBER_OF_LED =1000;
        // public static final int LED_PORT_ID2 = 8;
        // public static final int NUMBER_OF_LED2 =120;
    }
}
