// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class CmdShooterSetBedAngle extends Command
{
    private Shooter _shooter;
    private Rotation2d _bedAngle;

    public CmdShooterSetBedAngle(Shooter shooter, Rotation2d bedAngle)
    {
        _shooter = shooter;
        _bedAngle = bedAngle;
    }

    @Override
    public void initialize()
    {
        _shooter.setAngle(_bedAngle);
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}
