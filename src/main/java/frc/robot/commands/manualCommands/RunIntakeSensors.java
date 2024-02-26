// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manualCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/** Add your docs here. */
public class RunIntakeSensors extends Command{
    private final Intake m_IntakeSensors;
    private final double value;

    public RunIntakeSensors (Intake intake, double value) {
        this.m_IntakeSensors = intake;
        this.value = value;
        addRequirements(m_IntakeSensors);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_IntakeSensors.intake(value);
        
    }

    @Override
    public void end(boolean interrupted) {
        m_IntakeSensors.detectNote();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}