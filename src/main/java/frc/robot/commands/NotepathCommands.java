package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.notepath.Notepath;

public final class NotepathCommands
{
    private NotepathCommands()
    {
    }

    public static Command intakePickup(Notepath notepath)
    {
        return notepath.runOnce(notepath::setNotepathIntakePickupOn);
    }

    public static Command shooterPickup(Notepath notepath)
    {
        return notepath.runOnce(notepath::setNotepathShooterPickupOn);
    }

    public static Command startFeed(Notepath notepath)
    {
        return notepath.run(notepath::setFeedOn);
    }

    public static Command stopFeed(Notepath notepath)
    {
        return notepath.run(notepath::setOff);
    }

    public static Command reverseFeed(Notepath notepath)
    {
        return notepath.run(notepath::setReverse);
    }
}
