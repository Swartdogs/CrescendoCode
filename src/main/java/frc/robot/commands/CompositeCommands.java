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
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.climb.Climb;
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

    public static Command startNotepath(Notepath notepath, ShooterFlywheel shooterFlywheel, LED led)
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

    public static Command climbJoystick(Climb climb, ShooterBed shooterBed, BedAngle angle, DoubleSupplier leftSupplier, DoubleSupplier rightSupplier)
    {
        // @formatter:off
        return 
            Commands.sequence
            (
                ShooterBedCommands.setAngle(shooterBed, angle),
                Commands.waitUntil(() -> shooterBed.atSetpoint()),
                ClimbCommands.setVolts(climb, leftSupplier, rightSupplier)
            );
        // @formatter:on
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
