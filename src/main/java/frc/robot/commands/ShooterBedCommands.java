package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.ShooterBed;

public class ShooterBedCommands extends Command
{
    private ShooterBedCommands()
    {
    }

    public static Command setBedAngle(ShooterBed shooterBed, double bedAngle)
    {
        return Commands.run(() ->
        {
            shooterBed.setAngle(Rotation2d.fromDegrees(bedAngle));
        }, 
        
        shooterBed);
    }
}    
