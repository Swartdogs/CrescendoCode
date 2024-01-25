// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class CmdShooterStop extends Command
{
    private Shooter _shooter;

    public CmdShooterStop(Shooter shooter)
    {
        _shooter = shooter;
    }

    @Override
    public void initialize()
    {
        _shooter.setUpperVoltage(0);
        _shooter.setLowerVoltage(0);
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}
