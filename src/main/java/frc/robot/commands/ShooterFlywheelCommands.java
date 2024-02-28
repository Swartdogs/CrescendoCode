package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.shooter.ShooterFlywheel;

public final class ShooterFlywheelCommands
{
    private ShooterFlywheelCommands()
    {
    }

    public static Command start(ShooterFlywheel shooterFlywheel, double lowerVelocity, double upperVelocity)
    {
        return shooterFlywheel.runOnce(() -> shooterFlywheel.setVelocity(upperVelocity, lowerVelocity));
    }

    public static Command intake(ShooterFlywheel shooterFlywheel)
    {
        return shooterFlywheel.runOnce(shooterFlywheel::intake);
    }

    public static Command stop(ShooterFlywheel shooterFlywheel)
    {
        return shooterFlywheel.runOnce(shooterFlywheel::stop);
    }
}
