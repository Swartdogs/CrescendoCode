package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.notepath.Notepath;
import frc.robot.subsystems.shooter.ShooterBed;

public final class CompositeCommands
{
    private CompositeCommands()
    {
    }

    public static Command intakePickup(Intake intake, Notepath notepath)
    {
        return Commands.parallel(IntakeCommands.startIntake(intake), NotepathCommands.intakePickup(notepath)).andThen(Commands.waitUntil(() -> !notepath.hasNote())).finallyDo(() ->
        {
            intake.setIntakeOff();
            notepath.setOff();
        });
    }

    public static Command stopIntaking(Intake intake, Notepath notepath)
    {
        return Commands.parallel(IntakeCommands.stopIntake(intake), NotepathCommands.stopFeed(notepath));
    }

    public static Command shooterPickup(ShooterBed shooterBed, Notepath notepath)
    {
        return Commands.parallel(ShooterBedCommands.setBedAngle(shooterBed, Constants.ShooterBed.BED_PICKUP_ANGLE)).andThen(Commands.waitUntil(() -> !notepath.hasNote())).finallyDo(() -> notepath.setOff());
    }
}
