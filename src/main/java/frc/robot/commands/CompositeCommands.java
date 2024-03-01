package frc.robot.commands;

import java.util.Arrays;

import org.littletonrobotics.junction.inputs.LoggedDriverStation;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LED;
import frc.robot.subsystems.notepath.Notepath;
import frc.robot.subsystems.shooter.ShooterBed;
import frc.robot.subsystems.shooter.ShooterFlywheel;

public final class CompositeCommands
{
    private CompositeCommands()
    {
    }

    public static Command startShooter(Notepath notepath, ShooterFlywheel shooterFlywheel, LED led, double upperVelocity, double lowerVelocity)
    {
        return ShooterFlywheelCommands.shooterFlywheelShoot(shooterFlywheel, lowerVelocity, upperVelocity).andThen(Commands.waitUntil(() -> !notepath.hasNote())).finallyDo(() ->
        {
            shooterFlywheel.stop();
            LEDFillUp(led, Constants.LED.GREEN);
        });
    }

    public static Command startNotepath(Notepath notepath, ShooterFlywheel shooterFlywheel, LED led)
    {
        return Commands.waitUntil(() -> shooterFlywheel.atSpeed()).andThen(NotepathCommands.startFeed(notepath)).andThen(Commands.waitUntil(() -> notepath.sensorTripped())).andThen(Commands.waitUntil(() -> !notepath.sensorTripped()))
                .finallyDo(interrupted ->
                {
                    notepath.setOff();
                    notepath.setHasNote(!interrupted);
                }).onlyIf(() -> shooterFlywheel.isShooting());
    }

    public static Command stopShooter(ShooterFlywheel shooterFlywheel, Notepath notepath)
    {
        return Commands.parallel(ShooterFlywheelCommands.shooterFlywheelStop(shooterFlywheel), NotepathCommands.stopFeed(notepath));
    }

    public static Command intakePickup(Intake intake, Notepath notepath, ShooterBed shooterBed)
    {
        return Commands.parallel(IntakeCommands.startIntake(intake), NotepathCommands.intakePickup(notepath), ShooterBedCommands.setBedIntakePickupAngle(shooterBed)).andThen(Commands.waitUntil(() -> notepath.sensorTripped()))
                .finallyDo(interrupted ->
                {
                    intake.setIntakeOff();
                    notepath.setOff();
                    notepath.setHasNote(interrupted);
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
                    notepath.setHasNote(interrupted);
                }).unless(() -> notepath.hasNote());
    }

    public static Command LEDFillUp(LED led, Color color)
    {
        // @formatter:off
        int totalLEDs      = Constants.LED.NUM_LEDS;
        
        Command[] commandSequence = new Command[totalLEDs];
        return Commands.sequence
        (
            Commands.runOnce(() -> 
            {
                var initialPattern = led.getLEDs();
        
                for (int i = 0; i <= totalLEDs; i++)
                {
                    Color[] newPattern = new Color[totalLEDs];
                    Arrays.fill(newPattern, 0, i, color);
                    if (i < totalLEDs)
                    {
                        System.arraycopy(initialPattern, i, newPattern, i, totalLEDs - i);
                    }
                    commandSequence[i] = LEDCommands.setFrame(led, newPattern);
                }
            }),
            Commands.sequence(commandSequence)
        );
        // @formatter:on
    }

    // Flywheel No Longer Ready to Shoot
    public static Command LEDFillDown(LED led, Color color)
    {
        // @formatter.off
        int totalLEDs      = Constants.LED.NUM_LEDS;
        Command[]  commandSequence = new Command[totalLEDs];

        return Commands.sequence
        (
            Commands.runOnce(() ->
            {
                var initialPattern = led.getLEDs();

                for (int i = totalLEDs; i >= 0; i--)
                    {
                    Color[] newPattern = new Color[totalLEDs];
                    Arrays.fill(newPattern, i, totalLEDs, color);
                    System.arraycopy(initialPattern, 0, newPattern, 0, i);
                    commandSequence[totalLEDs - i] = LEDCommands.setFrame(led, newPattern);
                }
            }),
            Commands.sequence(commandSequence)
        );
        // @formatter.on
    }

    // Note in Path
    public static Command LEDBlinking(LED led, Color color)
    {
        return Commands.repeatingSequence(
                LEDCommands.setFrame(led, new Color[] { color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color }), Commands.waitSeconds(1),
                LEDCommands.setFrame(led, new Color[] { null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null }), Commands.waitSeconds(1)
        );
    }

    // Coming for Note
    public static Command LEDMovingStrip(LED led, Notepath notepath, Color color)
    {
        return Commands.repeatingSequence(
                LEDCommands.setFrame(led, new Color[] { color, color, color, color, color, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null }),
                LEDCommands.setFrame(led, new Color[] { null, color, color, color, color, color, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null }),
                LEDCommands.setFrame(led, new Color[] { null, null, color, color, color, color, color, null, null, null, null, null, null, null, null, null, null, null, null, null, null }),
                LEDCommands.setFrame(led, new Color[] { null, null, null, color, color, color, color, color, null, null, null, null, null, null, null, null, null, null, null, null, null }),
                LEDCommands.setFrame(led, new Color[] { null, null, null, null, color, color, color, color, color, null, null, null, null, null, null, null, null, null, null, null, null }),
                LEDCommands.setFrame(led, new Color[] { null, null, null, null, null, color, color, color, color, color, null, null, null, null, null, null, null, null, null, null, null }),
                LEDCommands.setFrame(led, new Color[] { null, null, null, null, null, null, color, color, color, color, color, null, null, null, null, null, null, null, null, null, null }),
                LEDCommands.setFrame(led, new Color[] { null, null, null, null, null, null, null, color, color, color, color, color, null, null, null, null, null, null, null, null, null }),
                LEDCommands.setFrame(led, new Color[] { null, null, null, null, null, null, null, null, color, color, color, color, color, null, null, null, null, null, null, null, null }),
                LEDCommands.setFrame(led, new Color[] { null, null, null, null, null, null, null, null, null, color, color, color, color, color, null, null, null, null, null, null, null }),
                LEDCommands.setFrame(led, new Color[] { null, null, null, null, null, null, null, null, null, null, color, color, color, color, color, null, null, null, null, null, null }),
                LEDCommands.setFrame(led, new Color[] { null, null, null, null, null, null, null, null, null, null, null, color, color, color, color, color, null, null, null, null, null }),
                LEDCommands.setFrame(led, new Color[] { null, null, null, null, null, null, null, null, null, null, null, null, color, color, color, color, color, null, null, null, null }),
                LEDCommands.setFrame(led, new Color[] { null, null, null, null, null, null, null, null, null, null, null, null, null, color, color, color, color, color, null, null, null }),
                LEDCommands.setFrame(led, new Color[] { null, null, null, null, null, null, null, null, null, null, null, null, null, null, color, color, color, color, color, null, null }),
                LEDCommands.setFrame(led, new Color[] { null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, color, color, color, color, color, null }),
                LEDCommands.setFrame(led, new Color[] { null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, color, color, color, color, color }),
                LEDCommands.setFrame(led, new Color[] { color, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, color, color, color, color }),
                LEDCommands.setFrame(led, new Color[] { color, color, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, color, color, color }),
                LEDCommands.setFrame(led, new Color[] { color, color, color, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, color, color }),
                LEDCommands.setFrame(led, new Color[] { color, color, color, color, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, color })
        ).unless(() -> notepath.hasNote());
    }

    public static Command LEDSetSolidColor(LED led, Color color)
    {
        return Commands.sequence(LEDCommands.setFrame(led, new Color[] { color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color }));
    }

    public static Command LEDAutonomous(LED led)
    {
        Color color;

        switch (LoggedDriverStation.getDSData().allianceStation)
        {
            case DriverStationJNI.kRed1AllianceStation:
            case DriverStationJNI.kRed2AllianceStation:
            case DriverStationJNI.kRed3AllianceStation:
                color = Constants.LED.RED;
                break;
            case DriverStationJNI.kBlue1AllianceStation:
            case DriverStationJNI.kBlue2AllianceStation:
            case DriverStationJNI.kBlue3AllianceStation:
                color = Constants.LED.BLUE;
                break;
            default:
                color = Constants.LED.ORANGE;
        }

        return CompositeCommands.LEDSetSolidColor(led, color);
    }

    public static Command LEDTeleop(LED led)
    {
        return CompositeCommands.LEDFillDown(led, Constants.LED.ORANGE).andThen(() -> led.switchDefaultCommand(CompositeCommands.LEDSetSolidColor(led, Constants.LED.ORANGE)));
    }

    public static Command LEDBlinkingTeleOp(LED led)
    {
        return Commands.runOnce(() -> led.switchDefaultCommand(CompositeCommands.LEDBlinking(led, Constants.LED.ORANGE)));
    }

    public static Command LEDPartyMode(LED led)
    {
        return Commands.runOnce(() -> Commands.repeatingSequence(LEDCommands.setFrame(led, led.getRandomColoring(Constants.LED.PINK, Constants.LED.ORANGE, Constants.LED.TEAL))));
    }
}
