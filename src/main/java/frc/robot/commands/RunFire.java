// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class RunFire extends Command {
  /** Creates a new RunFire. */
  public final Shooter m_Shooter;
  private final double valueShoot;
  private final Indexer m_Indexer;
  private final double valueIndex;
  public RunFire(Shooter fire, double valueShoot, Indexer index, double valueIndex) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Shooter = fire;
    this.valueShoot = valueShoot; 
    this.m_Indexer = index;
    this.valueIndex = valueIndex;
    addRequirements(m_Shooter, m_Indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Shooter.idleFire(0.4);
    m_Indexer.index(valueIndex);
    m_Shooter.fire(valueShoot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
