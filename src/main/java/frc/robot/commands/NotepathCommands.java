package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.notepath.Notepath;

public class NotepathCommands extends Command
{
    private NotepathCommands()
    {
    }

    public static Command intakePickup(Notepath notepath)
    {
        return Commands.run(() ->
        {
            notepath.setNotepathIntakePickupOn();
        }, 
        
        notepath);
    }

    public static Command shooterPickup(Notepath notepath)
    {
        return Commands.run(() ->
        {
            notepath.setNotepathShooterPickupOn();
        }, 
        
        notepath);
    }

    public static Command startFeed(Notepath notepath)
    {
        return Commands.run(() ->
        {
            notepath.setFeedOn();
        }, 
        
        notepath);
    }

    public static Command stopFeed(Notepath notepath)
    {
        return Commands.run(() ->
        {
            notepath.setOff();
        }, 
        
        notepath);
    }

    public static Command reverseFeed(Notepath notepath)
    {
        return Commands.run(() ->
        {
            notepath.setReverse();
        }, 
        
        notepath);
    }
}    
