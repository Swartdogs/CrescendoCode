package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.notepath.Notepath;
import frc.robot.subsystems.shooter.ShooterBed;
import frc.robot.subsystems.shooter.ShooterFlywheel;

public final class CompositeCommands
{
    private CompositeCommands()
    {
    }

    public static Command startShooter(ShooterFlywheel shooterFlywheel, Notepath notepath, double upperVelocity, double lowerVelocity)
    {
        return ShooterFlywheelCommands.shooterFlywheelShoot(shooterFlywheel, lowerVelocity, upperVelocity).andThen(Commands.waitUntil(() -> !notepath.hasNote())).finallyDo(() ->
        {
            shooterFlywheel.stop();
        });
    }

    public static Command startNotepath(Notepath notepath, ShooterFlywheel shooterFlywheel)
    {
        return Commands.waitUntil(() -> shooterFlywheel.atSpeed()).andThen(NotepathCommands.startFeed(notepath))
                .andThen(
                        Commands.either(
                                Commands.waitSeconds(0.75), Commands.waitUntil(() -> notepath.sensorTripped()).andThen(Commands.waitUntil(() -> !notepath.sensorTripped())),
                                () -> Constants.AdvantageKit.CURRENT_MODE == Constants.AdvantageKit.Mode.SIM
                        ).andThen(Commands.runOnce(() -> notepath.setHasNote(false))).finallyDo(() ->
                        {
                            notepath.setOff();
                        })
                ).onlyIf(() -> shooterFlywheel.isShooting());
    }

    public static Command stopShooter(ShooterFlywheel shooterFlywheel, Notepath notepath)
    {
        return Commands.parallel(ShooterFlywheelCommands.shooterFlywheelStop(shooterFlywheel), NotepathCommands.stopFeed(notepath));
    }

    public static Command intakePickup(Intake intake, Notepath notepath, ShooterBed shooterBed)
    {
        return Commands.parallel(IntakeCommands.startIntake(intake), NotepathCommands.intakePickup(notepath), ShooterBedCommands.setBedIntakePickupAngle(shooterBed))
                .andThen(Commands.either(Commands.waitSeconds(1), Commands.waitUntil(() -> notepath.sensorTripped()), () -> Constants.AdvantageKit.CURRENT_MODE == Constants.AdvantageKit.Mode.SIM)).finallyDo(interrupted ->
                {
                    intake.setIntakeOff();
                    notepath.setOff();
                    notepath.setHasNote(!interrupted);
                }).unless(() -> notepath.hasNote());
    }

    public static Command stopIntaking(Intake intake, Notepath notepath)
    {
        return Commands.parallel(IntakeCommands.stopIntake(intake), NotepathCommands.stopFeed(notepath));
    }

    public static Command shooterPickup(ShooterBed shooterBed, ShooterFlywheel shooterFlywheel, Notepath notepath)
    {
        return Commands.parallel(ShooterBedCommands.setBedShooterPickupAngle(shooterBed), ShooterFlywheelCommands.shooterFlywheelIntake(shooterFlywheel), NotepathCommands.reverseFeed(notepath))
                .andThen(Commands.waitUntil(() -> notepath.sensorTripped())).andThen(Commands.waitUntil(() -> !notepath.sensorTripped())).finallyDo(interrupted ->
                {
                    shooterFlywheel.stop();
                    notepath.setOff();
                    notepath.setHasNote(!interrupted);
                }).unless(() -> notepath.hasNote());
    }
}
