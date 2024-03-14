
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = 45.352; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class IntakeConstatnts
  {
    public static final int MOTOR_ID = 3;
    public static final int LEFT_SENSOR_ID = 0;
    public static final int RIGHT_SENSOR_ID = 1;
    public static final double INTAKE_SPEED = 0.5;
    public static final double NOTE_SETPOINT = 0.0;
    public static final double kP = 5.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFF = 0.0;
    public static final double MIN_INPUT = -1.0;
    public static final double MAX_INPUT = 1.0;
  }

  public static final class WristConstants
  {
    public static final int MOTOR_ID = 4;
    public static final double ZERO_OFFSET = 0.67;
    public static final double INTAKE_SETPOINT_POS = 0.162235856056213;
    public static final double SHOOTING_SETPOINT_POS = 0.364192873239517;
    public static final double AMP_SETPOINT_POS = 0.488;
    public static final double UP_LIMIT_POS = 0.7; // TODO: This is a guess!!
    public static final double DOWN_LIMIT_POS = 0.03; // TODO: This is a guess!!
    public static final double kP = 5.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFF = 0.0;
    public static final double MIN_INPUT = -0.5;
    public static final double MAX_INPUT = 0.5;
    // Below values are from Lil Miss, measured on 3/9/24
    public static final double WRIST_MIN_DOWN = 0.14; // Also for climbing.
    public static final double WRIST_MAX_UP = 0.82; // Also for shooting.
    public static final double WRIST_FURTHEST_DOWN_WHERE_TROLLEY_CAN_MOVE_FREELY = 0.46;
    public static final double WRIST_INTAKE_POSITION = 0.563;
  }

  public static final class TrolleyConstants
  {
    public static final int TROLLEY_MOTOR_ID = 5;
    public static final int TROLLEY_OUT_LIMIT_SWITCH_ID = 7;
    public static final int TROLLEY_IN_LIMIT_SWITCH_ID = 2;
    public static final int TROLLEY_POTENTIOMETER_ID = 3;
    public static final double TROLLEY_FORWARD_SPEED = 1.0;
    public static final double TROLLEY_REVERSE_SPEED = -1.0;
    public static final double HOME_SETPOINT_POS = 0.0;
    public static final double INTAKE_SETPOINT_POS = 107.47097778320312;
    public static final double AMP_SETPOINT_POS = 69;
    public static final double UP_LIMIT_POS = 90; // TODO: This is a guess!!
    public static final double DOWN_LIMIT_POS = -20; // TODO: This is a guess!!
    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFF = 0.0;
    public static final double MIN_INPUT = -0.3;
    public static final double MAX_INPUT = 0.3;
    // Below values measured on 3/9/24
    public static final double TROLLEY_MIN_IN = 0.70; // Flutters up to 0.73
    public static final double TROLLEY_SHOOTING_POSITION = 1.10; // Also the farthest back it can be when the wrist is up. Flutters up to 1.2.
    public static final double TROLLEY_MAX_OUT = 1.53; // Flutters between 1.53 & 1.57
    public static final double WRIST_POS_LOWER_LIMIT_WHILE_TROLLEY_DOWN = 0.7024;
    public static final double TROLLEY_POS_LOWEST_POINT_WRIST_CAN_MOVE = 4.185;  
    public static final double TROLLEY_INTAKE_POSITION = 1.53; // almost the same as max out                                               
    public static final double TROLLEY_IN_OUT_THRESHOLD = 1.16; // Can use this to decide if it's in or out
    public static final double TROLLEY_FURTHEST_IN_WHERE_PIVOT_CAN_MOVE_ALL_THE_WAY_UP = 1.33;
    public static final double TROLLEY_FURTHEST_IN_WHERE_PIVOT_CAN_CLEAR_BACK_BUMPER = 1.06;
  }

  public static final class PivotConstants
  { 
    public static final int MOTOR1_ID = 6; 
    public static final int MOTOR2_ID = 7; 
    public static final int PIVOT_ENCODER_ID = 9; 
    public static final double PIVOT_SPEED = 0.2;
    public static final double ZERO_OFFSET = 0.65362201634055;
    public static final double HOME_SETPOINT_POS = 0.558592213964805;
    public static final double INTAKE_SETPOINT_POS = 0.620574640514366;
    public static final double SUBWOFFER_SETPOINT_POS = 0.700878617521965;
    public static final double PODIUM_SETPOINT_POS = 0.0;
    public static final double WING_SETPOINT_POS = 0.0;
    public static final double AMP_SETPOINT_POS = 0.375752884393822;
    public static final double UP_LIMIT_POS = 0.8; // TODO: This is a guess!!
    public static final double DOWN_LIMIT_POS = -0.6; // TODO: This is a guess!!
    public static final double kP = 4.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFF = 0.0;
    public static final double MIN_INPUT = -0.3;
    public static final double MAX_INPUT = 0.3;
    // Below values measured on 3/9/24
    public static final double PIVOT_MIN_DOWN = 0.245; // This is the lowest point the "head" can be down (the head is the intake)
    public static final double PIVOT_BALANCED_FLAT = 0.405;
    public static final double PIVOT_CLIMB_POSITION = 0.669;
    public static final double PIVOT_MAX_UP = 0.70; // This is also the Amp scoring position.
    public static final double PIVOT_FURTHEST_DOWN_WHERE_TROLLEY_CAN_MOVE = 0.592305139807628;
    public static final double PIVOT_INTAKE_POSITION = 0.37;
    public static final double PIVOT_UP_FAR_ENOUGH_THAT_TROLLEY_COULD_HIT_BACK_BUMPER = 0.44;
    public static final double PIVOT_UP_FAR_ENOUGH_THAT_TROLLEY_COULD_HIT_UNDERBELLY = 0.48;
  }

  public static final class IndexerConstants
  {
    public static final int MOTOR_ID = 15;
    public static final double SHOT_RPM = 6000;
    public static final double IDLE_RPM = 1000;
    public static final double kP = 0.0004;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFF = 0.0;
    public static final double MIN_INPUT = -1;
    public static final double MAX_INPUT = 1;
  }

  public static final class ShooterConstants
  {
    public static final int TOP_MOTOR_ID = 8;
    public static final int BOTTOM_MOTOR_ID = 9;
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

  public static final class AutonConstants
  {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(0.4, 0, 0.01);
  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.01;
    public static final double LEFT_Y_DEADBAND  = 0.01;
    public static final double RIGHT_X_DEADBAND = 0.01;
    public static final double TURN_CONSTANT    = 6;
  }
}
