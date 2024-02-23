package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.notepath.Notepath;
import frc.robot.subsystems.notepath.Notepath.NotepathState;
import frc.robot.subsystems.shooter.ShooterBed;
import frc.robot.subsystems.shooter.ShooterFlywheel;

public final class CompositeCommands
{
    private CompositeCommands()
    {
    }

    public static Command startShooter(ShooterFlywheel shooterFlywheel, Notepath notepath, double upperVelocity, double lowerVelocity)
    {
        // @formatter:off
        return 
            Commands.sequence
            (
                ShooterFlywheelCommands.start(shooterFlywheel, lowerVelocity, upperVelocity),
                Commands.waitUntil(() -> !notepath.hasNote())
            )
            .finallyDo(() ->
            {
                shooterFlywheel.stop();
            });
        // @formatter:on
    }

    public static Command startNotepath(Notepath notepath, ShooterFlywheel shooterFlywheel)
    {
        // @formatter:off
        return            
            Commands.sequence
            (
                Commands.waitUntil(() -> shooterFlywheel.atSpeed()),
                NotepathCommands.feed(notepath),
                Commands.waitUntil(() -> notepath.sensorTripped()),
                Commands.waitUntil(() -> !notepath.sensorTripped()),
                Commands.runOnce(() -> notepath.setHasNote(false))
            )
            .finallyDo(() -> notepath.set(NotepathState.Off))
            .onlyIf(() -> shooterFlywheel.isShooting());
        // @formatter:on
    }

    public static Command stopShooter(ShooterFlywheel shooterFlywheel, Notepath notepath)
    {
        // @formatter:off
        return
            Commands.parallel
            (
                ShooterFlywheelCommands.stop(shooterFlywheel),
                NotepathCommands.stop(notepath)
            );
        // @formatter:on
    }

    public static Command intakePickup(Intake intake, Notepath notepath, ShooterBed shooterBed)
    {
        // @formatter:off
        return
            Commands.parallel
            (
                IntakeCommands.start(intake),
                NotepathCommands.intakeLoad(notepath),
                ShooterBedCommands.setAngle(shooterBed, ShooterBed.BedAngle.IntakeLoad)
            )
            .andThen(Commands.waitUntil(() -> notepath.sensorTripped()))
            .finallyDo(interrupted ->
            {
                intake.set(IntakeState.Off);
                notepath.set(NotepathState.Off);
                notepath.setHasNote(!interrupted);
            })
            .unless(() -> notepath.hasNote());
        // @formatter:on
    }

    public static Command stopIntaking(Intake intake, Notepath notepath)
    {
        // @formatter:off
        return
            Commands.parallel
            (
                IntakeCommands.stop(intake),
                NotepathCommands.stop(notepath)
            );
        // @formatter:on
    }

    public static Command shooterPickup(ShooterBed shooterBed, ShooterFlywheel shooterFlywheel, Notepath notepath)
    {
        // @formatter:off
        return
            Commands.parallel
            (
                ShooterBedCommands.setAngle(shooterBed, ShooterBed.BedAngle.ShooterLoad),
                ShooterFlywheelCommands.intake(shooterFlywheel),
                NotepathCommands.shooterLoad(notepath)
            )
            .andThen
            (
                Commands.waitUntil(() -> notepath.sensorTripped()),
                Commands.waitUntil(() -> !notepath.sensorTripped())
            )
            .finallyDo(interrupted ->
            {
                shooterFlywheel.stop();
                notepath.set(NotepathState.Off);
                notepath.setHasNote(!interrupted);
            })
            .unless(() -> notepath.hasNote());
        // @formatter:on
    }
}
