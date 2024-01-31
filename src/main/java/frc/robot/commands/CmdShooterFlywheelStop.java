// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterFlywheel;

public class CmdShooterFlywheelStop extends Command
{
    private ShooterFlywheel _shooterFlywheel;

    public CmdShooterFlywheelStop(ShooterFlywheel shooterFlywheel)
    {
        _shooterFlywheel = shooterFlywheel;
        addRequirements(_shooterFlywheel);
    }

    @Override
    public void initialize()
    {
        _shooterFlywheel.stopUpper();
        _shooterFlywheel.stopLower();
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}
