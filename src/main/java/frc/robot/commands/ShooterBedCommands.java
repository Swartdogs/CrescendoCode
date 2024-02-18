package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.shooter.ShooterBed;

public final class ShooterBedCommands
{
    private ShooterBedCommands()
    {
    }

    public static Command setAngle(ShooterBed shooterBed, double angle)
    {
        return shooterBed.runOnce(() -> shooterBed.setAngle(Rotation2d.fromDegrees(angle)));
    }

    public static Command setAngle(ShooterBed shooterBed, ShooterBed.BedAngle angle)
    {
        return shooterBed.runOnce(() -> shooterBed.setAngle(angle));
    }
}
