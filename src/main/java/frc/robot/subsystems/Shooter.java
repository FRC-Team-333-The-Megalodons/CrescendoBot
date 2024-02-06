// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// two motors 
package frc.robot.subsystems;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Shooter extends SubsystemBase{
    private CANSparkFlex shooterRight;
    private CANSparkFlex shooterLeft;
    
public Shooter() {
    shooterRight = new CANSparkFlex(8, MotorType.kBrushless);
    shooterRight.setIdleMode(IdleMode.kCoast);
    shooterLeft = new CANSparkFlex(9, MotorType.kBrushless);
    shooterLeft.setIdleMode(IdleMode.kCoast);
}
    public void idleFire(){shooterRight.set(0.4); shooterLeft.set(0.4);}
    public void fire(){shooterRight.set(1); shooterLeft.set(1);}
    public void fireStop(){shooterRight.set(0);shooterLeft.set(0);}
}
