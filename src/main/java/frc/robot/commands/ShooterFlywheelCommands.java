package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.ShooterFlywheel;

public class ShooterFlywheelCommands extends Command
{
    private ShooterFlywheelCommands()
    {
    }

    public static Command shooterFlywheelShoot(ShooterFlywheel shooterFlywheel, double lowerVelocity, double upperVelocity)
    {
        return Commands.run(() ->
        {
            shooterFlywheel.setUpperVelocity(upperVelocity);
            shooterFlywheel.setLowerVelocity(lowerVelocity);
        }, 

        shooterFlywheel);
    }

    public static Command shooterFlywheelStop(ShooterFlywheel shooterFlywheel)
    {
        return Commands.run(() ->
        {
            shooterFlywheel.stop();
        }, 

        shooterFlywheel);
    }
}    
