package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterBed;

public class ShooterBedCommands
{
    private ShooterBedCommands()
    {
    }

    public static Command setBedAngle(ShooterBed shooterBed, double bedAngle)
    {
        return shooterBed.runOnce(() -> 
            shooterBed.setAngle(Rotation2d.fromDegrees(bedAngle)));
    }
}    