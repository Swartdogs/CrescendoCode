package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.notepath.Notepath;

public final class CompositeCommands
{
    private CompositeCommands()
    {
    }

    public static Command intakePickup(Intake intake, Notepath notepath)
    {
        return Commands.startEnd(
            () -> Commands.parallel(IntakeCommands.startIntake(intake), NotepathCommands.intakePickup(notepath)), 
            () -> Commands.parallel(IntakeCommands.stopIntake(intake), (NotepathCommands.stopFeed(notepath))));
    }

    public static Command stopIntaking(Intake intake, Notepath notepath)
    {
        return Commands.runOnce(
            () -> Commands.parallel(IntakeCommands.stopIntake(intake), NotepathCommands.stopFeed(notepath)));
    }
}
