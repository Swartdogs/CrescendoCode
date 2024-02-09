package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.notepath.Notepath;
import frc.robot.subsystems.shooter.ShooterBed;
import frc.robot.subsystems.shooter.ShooterFlywheel;

public final class CompositeCommands
{
    private CompositeCommands()
    {
    }

    public static Command intakePickup(Intake intake, Notepath notepath, ShooterBed shooterBed)
    {
        return Commands.parallel(IntakeCommands.startIntake(intake), NotepathCommands.intakePickup(notepath), ShooterBedCommands.setBedIntakePickupAngle(shooterBed)).andThen(Commands.waitUntil(() -> notepath.hasNote())).finallyDo(() ->
        {
            intake.setIntakeOff();
            notepath.setOff();
        }).unless(() -> notepath.hasNote());
    }

    public static Command stopIntaking(Intake intake, Notepath notepath)
    {
        return Commands.parallel(IntakeCommands.stopIntake(intake), NotepathCommands.stopFeed(notepath));
    }

    public static Command shooterPickup(ShooterBed shooterBed, ShooterFlywheel shooterFlywheel, Notepath notepath)
    {
        return Commands.parallel(
                ShooterBedCommands.setBedShooterPickupAngle(shooterBed),
                ShooterFlywheelCommands.shooterFlywheelIntake(shooterFlywheel), NotepathCommands.reverseFeed(notepath)
        ).andThen(Commands.waitUntil(() -> notepath.hasNote())).finallyDo(() ->
        {
            shooterFlywheel.stop();
            notepath.setOff();
        });
    }
}
