package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterFlywheel;

public class ShooterFlywheelCommands
{
    private ShooterFlywheelCommands()
    {
    }

    public static Command shooterFlywheelShoot(ShooterFlywheel shooterFlywheel, double lowerVelocity, double upperVelocity)
    {
        return shooterFlywheel.runOnce(() ->
        {
            shooterFlywheel.setUpperVelocity(upperVelocity);
            shooterFlywheel.setLowerVelocity(lowerVelocity);
        });
    }

    public static Command shooterFlywheelStop(ShooterFlywheel shooterFlywheel)
    {
        return shooterFlywheel.runOnce(shooterFlywheel::stop);
    }
}    
