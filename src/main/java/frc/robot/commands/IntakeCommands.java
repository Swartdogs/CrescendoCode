package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommands
{
    private IntakeCommands()
    {
    }

    public static Command startIntake(Intake intake)
    {
        return intake.runOnce(intake::setIntakeOn);
    }

    public static Command stopIntake(Intake intake)
    {
        return intake.runOnce(intake::setIntakeOff);
    }

    public static Command reverseIntake(Intake intake)
    {
        return intake.runOnce(intake::setIntakeReverse);
    }
}    
