// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

public class CmdIntakeStopIntake extends Command
{
    private final Intake _intakeSubsystem;

    public CmdIntakeStopIntake(Intake intakeSubsystem)
    {
        _intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void initialize()
    {
        _intakeSubsystem.setIntakeOff();
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}
