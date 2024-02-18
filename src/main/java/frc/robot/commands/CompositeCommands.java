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
        return ShooterFlywheelCommands.shooterFlywheelShoot(shooterFlywheel, lowerVelocity, upperVelocity).andThen(Commands.waitUntil(() -> !notepath.hasNote())).finallyDo(() ->
        {
            shooterFlywheel.stop();
        });
    }

    public static Command startNotepath(Notepath notepath, ShooterFlywheel shooterFlywheel)
    {
        return Commands.waitUntil(() -> shooterFlywheel.atSpeed()).andThen(NotepathCommands.startFeed(notepath)).andThen(Commands.waitUntil(() -> notepath.sensorTripped())).andThen(Commands.waitUntil(() -> !notepath.sensorTripped()))
                .andThen(Commands.runOnce(() -> notepath.setHasNote(false))).finallyDo(() ->
                {
                    notepath.setOff();
                }).onlyIf(() -> shooterFlywheel.isShooting());
    }

    public static Command stopShooter(ShooterFlywheel shooterFlywheel, Notepath notepath)
    {
        return Commands.parallel(ShooterFlywheelCommands.shooterFlywheelStop(shooterFlywheel), NotepathCommands.stopFeed(notepath));
    }

    public static Command intakePickup(Intake intake, Notepath notepath, ShooterBed shooterBed)
    {
        return Commands.parallel(IntakeCommands.start(intake), NotepathCommands.intakeLoad(notepath), ShooterBedCommands.setAngle(shooterBed, ShooterBed.BedAngle.IntakeLoad)).andThen(Commands.waitUntil(() -> notepath.sensorTripped()))
                .finallyDo(interrupted ->
                {
                    intake.set(IntakeState.Off);
                    notepath.set(NotepathState.Off);
                    notepath.setHasNote(!interrupted);
                }).unless(() -> notepath.hasNote());
    }

    public static Command stopIntaking(Intake intake, Notepath notepath)
    {
        return Commands.parallel(IntakeCommands.stop(intake), NotepathCommands.stop(notepath));
    }

    public static Command shooterPickup(ShooterBed shooterBed, ShooterFlywheel shooterFlywheel, Notepath notepath)
    {
        return Commands.parallel(ShooterBedCommands.setAngle(shooterBed, ShooterBed.BedAngle.ShooterLoad), ShooterFlywheelCommands.shooterFlywheelIntake(shooterFlywheel), NotepathCommands.shooterLoad(notepath))
                .andThen(Commands.waitUntil(() -> notepath.sensorTripped())).andThen(Commands.waitUntil(() -> !notepath.sensorTripped())).finallyDo(interrupted ->
                {
                    shooterFlywheel.stop();
                    notepath.set(NotepathState.Off);
                    notepath.setHasNote(!interrupted);
                }).unless(() -> notepath.hasNote());
    }
}
