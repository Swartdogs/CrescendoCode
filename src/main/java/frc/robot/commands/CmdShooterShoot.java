// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class CmdShooterShoot extends Command
{
    private Shooter _shooter;

    private double _upperVoltage;
    private double _lowerVoltage;

    public CmdShooterShoot(Shooter shooter, double lowerVoltage, double upperVoltage)
    {
        _shooter = shooter;
        _lowerVoltage = lowerVoltage;
        _upperVoltage = upperVoltage;
    }

    @Override
    public void initialize()
    {
        _shooter.setVoltage(_upperVoltage, _lowerVoltage);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        _shooter.setVoltage(0,0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
