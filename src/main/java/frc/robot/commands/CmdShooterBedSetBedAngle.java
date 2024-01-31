// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterBed;

public class CmdShooterBedSetBedAngle extends Command
{
    private ShooterBed _shooterBed;
    private Rotation2d _bedAngle;

    public CmdShooterBedSetBedAngle(ShooterBed shooterBed, Rotation2d bedAngle)
    {
        _shooterBed = shooterBed;
        _bedAngle   = bedAngle;
        addRequirements(_shooterBed);
    }

    @Override
    public void initialize()
    {
        _shooterBed.setAngle(_bedAngle);
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}
