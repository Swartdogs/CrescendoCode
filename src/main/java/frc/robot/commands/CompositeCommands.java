// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.notepath.Notepath;
import frc.robot.subsystems.shooter.ShooterFlywheel;

public final class CompositeCommands 
{
    private CompositeCommands()
    {
    }

    public static Command start(ShooterFlywheel shooterFlywheel, Notepath notepath, double upperVelocity, double lowerVelocity)
    {
        return Commands.parallel(NotepathCommands.startFeed(notepath), ShooterFlywheelCommands.shooterFlywheelShoot(shooterFlywheel, lowerVelocity, upperVelocity)).finallyDo(() ->
        {
            shooterFlywheel.setLowerVelocity(0);
            shooterFlywheel.setUpperVelocity(0);
            notepath.setOff();
        });
    }
}