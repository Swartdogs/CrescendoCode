package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.climb.Climb;
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

    public static class Autonomous
    {
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
                    )
                )
                .unless(() -> notepath.hasNote());
            // @formatter:on
        }

        private static Command load(Intake intake, Notepath notepath)
        {
            // @formatter:off
            return 
                Commands.runOnce(() -> {}, intake, notepath)
                .andThen
                (
                    Commands.either
                    (
                        Commands.waitSeconds(1),
                        Commands.waitUntil(() -> notepath.sensorTripped()), 
                        () -> Constants.AdvantageKit.CURRENT_MODE == Constants.AdvantageKit.Mode.SIM
                    )
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

        public static Command startShooter(ShooterFlywheel shooterFlywheel, Notepath notepath, double upperVelocity, double lowerVelocity)
        {
            // @formatter:off
            return 
                ShooterFlywheelCommands.start(shooterFlywheel, lowerVelocity, upperVelocity)
                .onlyIf(()-> notepath.hasNote());
            // @formatter:on
        }

        public static Command setBedAngle(ShooterBed shooterBed, BedAngle bedAngle)
        {
            // @formatter:off
            return 
                Commands.sequence
                (
                    ShooterBedCommands.setAngle(shooterBed, bedAngle),
                    Commands.waitUntil(() -> shooterBed.atSetpoint())
                );
            // @formatter:on
        }

        public static Command setBedAngle(ShooterBed shooterBed, double bedAngle)
        {
            // @formatter:off
            return 
                Commands.sequence
                (
                    ShooterBedCommands.setAngle(shooterBed, bedAngle),
                    Commands.waitUntil(() -> shooterBed.atSetpoint())
                );
            // @formatter:on
        }
    }

    public static class General
    {
        public static Command startNotepath(ShooterBed shooterBed, Notepath notepath, ShooterFlywheel shooterFlywheel)
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
                    Commands.runOnce(() -> notepath.setHasNote(false)),
                    ShooterBedCommands.setAngle(shooterBed, ShooterBed.BedAngle.Stow)
                )
                .finallyDo(interrupted -> 
                {
                    if (!interrupted)
                    {
                        notepath.set(NotepathState.Off);
                        shooterFlywheel.stop();
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

        public static Command setHasNote(Notepath notepath, boolean hasNote)
        {
            return Commands.runOnce(() -> notepath.setHasNote(hasNote)).ignoringDisable(true);
        }
    }

    public static class Teleop
    {
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
                        Commands.waitUntil(() -> notepath.sensorTripped()),
                        ShooterBedCommands.setAngle(shooterBed, ShooterBed.BedAngle.Stow)
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

        public static Command startShooter(ShooterFlywheel shooterFlywheel, Notepath notepath, double upperVelocity, double lowerVelocity)
        {
            // @formatter:off
            return 
                Commands.sequence
                (
                    ShooterFlywheelCommands.start(shooterFlywheel, lowerVelocity, upperVelocity)
                )
                .onlyIf(() -> notepath.hasNote());
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
                    ShooterFlywheelCommands.start(shooterFlywheel, lowerVelocity, upperVelocity)
                )
                .onlyIf(()-> notepath.hasNote());
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
                    ShooterFlywheelCommands.start(shooterFlywheel, lowerVelocity, upperVelocity)
                )
                .onlyIf(()-> notepath.hasNote());
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
                    Commands.waitUntil(() -> !notepath.sensorTripped()),
                    ShooterBedCommands.setAngle(shooterBed, ShooterBed.BedAngle.Stow)
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
            // @formatter:off
            return 
                Commands.parallel
                (
                    NotepathCommands.shooterLoad(notepath), 
                    ShooterFlywheelCommands.intake(shooterFlywheel)
                )
                .andThen(Commands.idle(notepath, shooterFlywheel))
                .finallyDo(() ->
                {
                    notepath.set(NotepathState.Off);
                    shooterFlywheel.stop();
                });
            // @formatter:on
        }

        public static Command climbJoystick(Climb climb, ShooterBed shooterBed, BedAngle angle, DoubleSupplier leftSupplier, DoubleSupplier rightSupplier)
        {
            // @formatter:off
            return 
                Commands.sequence
                (
                    ShooterBedCommands.setAngle(shooterBed, angle),
                    Commands.waitUntil(() -> shooterBed.atSetpoint()),
                    ClimbCommands.setVolts(climb, leftSupplier, rightSupplier)
                )
                .finallyDo(() -> climb.stop());
            // @formatter:on
        }

        public static Command climbJoystick(Climb climb, DoubleSupplier leftSupplier, DoubleSupplier rightSupplier)
        {
            // @formatter:off
            return 
                ClimbCommands.setVolts(climb, leftSupplier, rightSupplier)
                .finallyDo(() -> climb.stop());
            // @formatter:on
        }

        public static Command setBedVolts(ShooterBed shooterBed, double volts)
        {
            return ShooterBedCommands.setVolts(shooterBed, volts).andThen(Commands.idle(shooterBed)).finallyDo(() -> shooterBed.setVolts(0));
        }
    }
}
