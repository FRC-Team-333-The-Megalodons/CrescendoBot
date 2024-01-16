// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

public class Constants {

    // Spark Flex Config
    public static final IdleMode driveMotorIdleMode = IdleMode.kCoast;
    public static final IdleMode steerMotorIdleMode = IdleMode.kBrake;

    public static final class ModuleConstants {

        public static final boolean encoderInvert = true;

        // Drive motor conversion factors
        public static final double driveMotorFreeSpeed = 0.0; // RPMs
    }

    /* Mod 0: Front Left */ 
    public static final int frontLeftDriveID = 11;
    public static final int frontLeftSteerID = 21;
    public static final int frontLeftEncoderID = 31;

    /* Mod 0: Front Left */ 
    public static final int frontRightDriveID = 12;
    public static final int frontRightSteerID = 22;
    public static final int frontRightEncoderID = 32;

    /* Mod 0: Front Left */ 
    public static final int rearLeftDriveID = 13;
    public static final int rearLeftSteerID = 23;
    public static final int rearLeftEncoderID = 33;

    /* Mod 0: Front Left */ 
    public static final int rearRightDriveID = 14;
    public static final int rearRightSteerID = 24;
    public static final int rearRightEncoderID = 34;
}
