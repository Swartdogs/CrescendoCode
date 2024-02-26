package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.notepath.Notepath;
import frc.robot.subsystems.notepath.Notepath.NotepathState;
import frc.robot.subsystems.shooter.ShooterBed;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterBed.BedAngle;

public final class CompositeCommands
{
    private CompositeCommands()
    {
    }

    public static Command startShooter(ShooterFlywheel shooterFlywheel, double upperVelocity, double lowerVelocity)
    {
        // @formatter:off
        return 
                ShooterFlywheelCommands.start(shooterFlywheel, lowerVelocity, upperVelocity);
        // @formatter:on
    }

    public static Command startShooter(ShooterFlywheel shooterFlywheel, Notepath notepath, ShooterBed shooterBed, double upperVelocity, double lowerVelocity, double shootAngle)
    {
        // @formatter:off
        return 
            Commands.sequence
            (
                ShooterBedCommands.setAngle(shooterBed, shootAngle),
                Commands.waitUntil(() -> shooterBed.atSetpoint()),
                ShooterFlywheelCommands.start(shooterFlywheel, lowerVelocity, upperVelocity),
                Commands.waitUntil(() -> !notepath.hasNote())
            )
            .finallyDo(() ->
            {
                shooterFlywheel.stop();
            });
        // @formatter:on
    }

    public static Command startShooter(ShooterFlywheel shooterFlywheel, Notepath notepath, ShooterBed shooterBed, double upperVelocity, double lowerVelocity, BedAngle shootAngle)
    {
        // @formatter:off
        return 
            Commands.sequence
            (
                ShooterBedCommands.setAngle(shooterBed, shootAngle),
                Commands.waitUntil(() -> shooterBed.atSetpoint()),
                ShooterFlywheelCommands.start(shooterFlywheel, lowerVelocity, upperVelocity),
                Commands.waitUntil(() -> !notepath.hasNote())
            )
            .finallyDo(() ->
            {
                shooterFlywheel.stop();
            });
        // @formatter:on
    }

    public static Command runShooter(ShooterFlywheel shooterFlywheel, Notepath notepath, DoubleSupplier velocitySupplier)
    {
        // @formatter:off
        return
            shooterFlywheel.run(() -> 
            {
                var velocity = velocitySupplier.getAsDouble();
                shooterFlywheel.setLowerVelocity(velocity);
                shooterFlywheel.setUpperVelocity(velocity);
            })
            .until(() -> !notepath.hasNote())
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
                Commands.either
                (
                    Commands.waitSeconds(1), 
                    Commands.sequence
                    (
                        Commands.waitUntil(() -> notepath.sensorTripped()),
                        Commands.waitUntil(() -> !notepath.sensorTripped())
                    ), 
                    () -> Constants.AdvantageKit.CURRENT_MODE == Constants.AdvantageKit.Mode.SIM
                ),
                Commands.runOnce(() -> notepath.setHasNote(false))
            )
            .finallyDo(interrupted -> 
            {
                if (!interrupted)
                {
                    notepath.set(NotepathState.Off);
                }
            })
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
            ShooterBedCommands.setAngle(shooterBed, ShooterBed.BedAngle.IntakeLoad)
            .andThen
            (
                Commands.waitUntil(() -> shooterBed.atSetpoint()),
                Commands.parallel
                (
                    IntakeCommands.start(intake),
                    NotepathCommands.intakeLoad(notepath)
                ),
                Commands.waitUntil(() -> notepath.sensorTripped())
            )
            .finallyDo(interrupted ->
            {
                intake.set(IntakeState.Off);
                notepath.set(NotepathState.Off);
                notepath.setHasNote(!interrupted);
            })
            .unless(() -> notepath.hasNote());
        // @formatter:on
    }

    private static Command load(Intake intake, Notepath notepath)
    {
        // @formatter:off
        return 
            Commands.either
            (
                Commands.waitSeconds(1),
                Commands.waitUntil(() -> notepath.sensorTripped()), 
                () -> Constants.AdvantageKit.CURRENT_MODE == Constants.AdvantageKit.Mode.SIM
            );
        // @formatter:on
    }

    public static Command loadWhileStopped(Intake intake, Notepath notepath)
    {
        // @formatter:off
        return
            load(intake, notepath)
            .finallyDo(interrupted ->
            {
                intake.set(IntakeState.Off);
                notepath.set(NotepathState.Off);
                notepath.setHasNote(!interrupted);
            })
            .unless(() -> notepath.hasNote());
        // @formatter:on
    }

    public static Command loadInMotion(Intake intake, Notepath notepath)
    {
        // @formatter:off
        return
            load(intake, notepath)
            .finallyDo(interrupted ->
            {
                if (!interrupted)
                {
                    intake.set(IntakeState.Off);
                    notepath.set(NotepathState.Off);
                }
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

            ShooterBedCommands.setAngle(shooterBed, ShooterBed.BedAngle.ShooterLoad)
            .andThen
            (
                Commands.waitUntil(() -> shooterBed.atSetpoint()),
                Commands.parallel
                (
                    ShooterFlywheelCommands.intake(shooterFlywheel),
                    NotepathCommands.shooterLoad(notepath)
                ),
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

    public static Command suckIn(Notepath notepath, ShooterFlywheel shooterFlywheel)
    {
        return Commands.parallel(NotepathCommands.shooterLoad(notepath), ShooterFlywheelCommands.intake(shooterFlywheel));
    }
}
