// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterFlywheel;

public class CmdShooterFlywheelShoot extends Command
{
    private ShooterFlywheel _shooterFlywheel;
    private double          _upperVelocity;
    private double          _lowerVelocity;

    public CmdShooterFlywheelShoot(ShooterFlywheel shooterFlywheel, double lowerVelocity, double upperVelocity)
    {
        _shooterFlywheel = shooterFlywheel;
        _lowerVelocity   = lowerVelocity;
        _upperVelocity   = upperVelocity;
        addRequirements(_shooterFlywheel);
    }

    @Override
    public void initialize()
    {
        _shooterFlywheel.setUpperVelocity(_upperVelocity);
        _shooterFlywheel.setLowerVelocity(_lowerVelocity);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        _shooterFlywheel.setUpperVelocity(0);
        _shooterFlywheel.setLowerVelocity(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
