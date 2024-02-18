package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.notepath.Notepath;
import frc.robot.subsystems.notepath.Notepath.NotepathState;

public final class NotepathCommands
{
    private NotepathCommands()
    {
    }

    public static Command intakeLoad(Notepath notepath)
    {
        return notepath.runOnce(() -> notepath.set(NotepathState.IntakeLoad));
    }

    public static Command shooterLoad(Notepath notepath)
    {
        return notepath.runOnce(() -> notepath.set(NotepathState.ShooterLoad));
    }

    public static Command feed(Notepath notepath)
    {
        return notepath.runOnce(() -> notepath.set(NotepathState.Feed));
    }

    public static Command stop(Notepath notepath)
    {
        return notepath.runOnce(() -> notepath.set(NotepathState.Off));
    }
}
