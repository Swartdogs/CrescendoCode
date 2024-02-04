package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommands extends Command
{
    private IntakeCommands()
    {
    }

    public static Command startIntake(Intake intake)
    {
        return Commands.run(() ->
        {
            intake.setIntakeOn();
        }, 
        
        intake);
    }

    public static Command stopIntake(Intake intake)
    {
        return Commands.run(() ->
        {
            intake.setIntakeOff();
        }, 

        intake);
    }

    public static Command reverseIntake(Intake intake)
    {
        return Commands.run(() ->
        {
            intake.setIntakeReverse();
        }, 
        
        intake);
    }
}    
