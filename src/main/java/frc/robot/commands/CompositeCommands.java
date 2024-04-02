package frc.robot.commands;

import java.util.Arrays;

import org.littletonrobotics.junction.inputs.LoggedDriverStation;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.stream.IntStream;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LED;

import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.notepath.Notepath;
import frc.robot.subsystems.notepath.Notepath.NotepathState;
import frc.robot.subsystems.shooter.ShooterBed;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterBed.BedAngle;
import static frc.robot.Constants.LED.*;
import frc.robot.util.DeferredInstantCommand;

public final class CompositeCommands
{
    private CompositeCommands()
    {
    }

    public static class Autonomous
    {
        public static Command startNotepath(Notepath notepath)
        {
            return Commands.sequence(
                    new DeferredInstantCommand(() -> Commands.sequence(NotepathCommands.feed(notepath), Commands.waitUntil(() -> !notepath.sensorTripped())).finallyDo(() -> notepath.set(NotepathState.Off))),
                    Commands.waitUntil(() -> !notepath.sensorTripped())
            );
        }

        public static Command intakePickup(Intake intake, Notepath notepath, ShooterBed shooterBed)
        {
            // @formatter:off
            return ShooterBedCommands.setAngle(shooterBed, ShooterBed.BedAngle.IntakeLoad)
                .andThen
                (
                    Commands.waitUntil(() -> shooterBed.atSetpoint()),
                    Commands.parallel
                    (
                        IntakeCommands.start(intake),
                        new DeferredInstantCommand(() -> NotepathCommands.intakeLoad(notepath))
                    )
                )
                .unless(() -> notepath.hasNote());
            // @formatter:on)
        }

        public static Command load(Notepath notepath)
        {
            return new DeferredInstantCommand(() -> Commands.sequence(NotepathCommands.intakeLoad(notepath), Commands.waitUntil(() -> notepath.sensorTripped()), NotepathCommands.stop(notepath)));
        }

        public static Command startShooter(ShooterFlywheel shooterFlywheel, double upperVelocity, double lowerVelocity)
        {
            // @formatter:off
            return Commands.sequence
            (
                ShooterFlywheelCommands.start(shooterFlywheel, lowerVelocity, upperVelocity),
                Commands.waitUntil(() -> shooterFlywheel.atSpeed())
            );
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
                    new DeferredInstantCommand(() -> NotepathCommands.feed(notepath)),
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
        public static Command intakePickup(Intake intake, Notepath notepath, ShooterBed shooterBed, GenericHID controller)
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
                    ShooterBedCommands.setAngle(shooterBed, ShooterBed.BedAngle.Stow),
                    rumble(controller)
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

        public static Command shooterPickup(ShooterBed shooterBed, ShooterFlywheel shooterFlywheel, Notepath notepath, GenericHID controller)
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
                    ShooterBedCommands.setAngle(shooterBed, ShooterBed.BedAngle.Stow),
                    rumble(controller)
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

        public static Command setBedVolts(ShooterBed shooterBed, DoubleSupplier ySupplier)
        {
            return ShooterBedCommands.setVolts(shooterBed, ySupplier).finallyDo(() -> shooterBed.setAngle(shooterBed.getBedAngle()));
        }

        public static Command driveAtOrientation(Drive drive, Dashboard dashboard, DoubleSupplier xSupplier, DoubleSupplier ySupplier, BooleanSupplier robotCentric, double blueSetpoint, double redSetpoint, double maxSpeed)
        {
            return Commands.either(
                    DriveCommands.driveAtOrientation(drive, dashboard, xSupplier, ySupplier, robotCentric, blueSetpoint, maxSpeed),
                    DriveCommands.driveAtOrientation(drive, dashboard, xSupplier, ySupplier, robotCentric, redSetpoint, maxSpeed), () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue
            );
        }

        public static Command redAmpOrSubwoofer(Drive drive, Dashboard dashboard, DoubleSupplier xSupplier, DoubleSupplier ySupplier, BooleanSupplier robotCentric, double maxSpeed)
        {
            return Commands.either(
                    DriveCommands.driveAtOrientation(drive, dashboard, xSupplier, ySupplier, robotCentric, 0, maxSpeed), // subwoofer shot
                    DriveCommands.driveAtOrientation(drive, dashboard, xSupplier, ySupplier, robotCentric, -90, maxSpeed), // amp shot
                    () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue
            );
        }

        public static Command blueAmpOrSubwoofer(Drive drive, Dashboard dashboard, DoubleSupplier xSupplier, DoubleSupplier ySupplier, BooleanSupplier robotCentric, double maxSpeed)
        {
            return Commands.either(
                    DriveCommands.driveAtOrientation(drive, dashboard, xSupplier, ySupplier, robotCentric, -90, maxSpeed), // subwoofer shot
                    DriveCommands.driveAtOrientation(drive, dashboard, xSupplier, ySupplier, robotCentric, 180, maxSpeed), // amp shot
                    () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue
            );
        }

        public static Command redAmpOrPodium(ShooterFlywheel shooterFlywheel, Notepath notepath, ShooterBed shooterBed)
        {
            return Commands.either(
                    CompositeCommands.Teleop.startShooter(shooterFlywheel, notepath, shooterBed, 4500, 4500, ShooterBed.BedAngle.PodiumShot),
                    CompositeCommands.Teleop.startShooter(shooterFlywheel, notepath, shooterBed, 250, 2500, ShooterBed.BedAngle.AmpShot), () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue
            );
        }

        public static Command blueAmpOrPodium(ShooterFlywheel shooterFlywheel, Notepath notepath, ShooterBed shooterBed)
        {
            return Commands.either(
                    CompositeCommands.Teleop.startShooter(shooterFlywheel, notepath, shooterBed, 250, 2500, ShooterBed.BedAngle.AmpShot),
                    CompositeCommands.Teleop.startShooter(shooterFlywheel, notepath, shooterBed, 4500, 4500, ShooterBed.BedAngle.PodiumShot),
                    () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue
            );
        }

        public static Command rumble(GenericHID controller)
        {
            return new DeferredInstantCommand(
                    () -> Commands.sequence(Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 1)), Commands.waitSeconds(0.5), Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 0)))
            );
        }

        public static Command runThrough(BooleanSupplier buttonPressed, Climb climb, Drive drive, Intake intake, Notepath notepath, ShooterFlywheel shooterFlywheel, ShooterBed shooterBed, Dashboard dashboard)
        {
            // @formatter:off
            return Commands.sequence
            (
                    Commands.waitUntil(buttonPressed), ClimbCommands.setHeight(climb, Constants.Climb.MIN_EXTENSION), // Climb
                    Commands.waitUntil(buttonPressed), ClimbCommands.setHeight(climb, Constants.Climb.LEFT_MAX_EXTENSION), // Climb
                    Commands.waitUntil(buttonPressed), ClimbCommands.setHeight(climb, Constants.Climb.RIGHT_MAX_EXTENSION), //C

                    Commands.deadline(Commands.waitUntil(buttonPressed), DriveCommands.joystickDrive(drive, () -> 0.5, () -> 0, () -> 0, () -> true, dashboard)), //Forward
                    Commands.deadline(Commands.waitUntil(buttonPressed), DriveCommands.joystickDrive(drive, () -> -0.5, () -> 0, () -> 0, () -> true, dashboard)), //Backwards
                    Commands.deadline(Commands.waitUntil(buttonPressed), DriveCommands.joystickDrive(drive, () -> 0.0, () -> 0.5, () -> 0, () -> true, dashboard)),  //Left or Right
                    Commands.deadline(Commands.waitUntil(buttonPressed), DriveCommands.joystickDrive(drive, () -> 0.0, () -> -0.5, () -> 0, () -> true, dashboard)), //Left or Right
                    Commands.deadline(Commands.waitUntil(buttonPressed), DriveCommands.joystickDrive(drive, () -> 0.0, () -> 0, () -> 0.5, () -> true, dashboard)), //Rotate Left or Right
                    Commands.deadline(Commands.waitUntil(buttonPressed), DriveCommands.joystickDrive(drive, () -> 0.5, () -> 0, () -> -0.5, () -> true, dashboard)), //Rotate Left or Right

                    Commands.waitUntil(buttonPressed), IntakeCommands.start(intake), // Intake

                    Commands.waitUntil(buttonPressed), NotepathCommands.intakeLoad(notepath),  // Notepath
                    Commands.waitUntil(buttonPressed), NotepathCommands.shooterLoad(notepath), // Notepath

                    Commands.waitUntil(buttonPressed), ShooterFlywheelCommands.start(shooterFlywheel, 4000, 4000), // Flywheel
                    Commands.waitUntil(buttonPressed), ShooterFlywheelCommands.intake(shooterFlywheel), // Flywheel

                    Commands.waitUntil(buttonPressed), ShooterBedCommands.setAngle(shooterBed, Constants.ShooterBed.BED_PODIUM_SHOT_ANGLE.getDegrees()), // Bed
                    Commands.waitUntil(buttonPressed), ShooterBedCommands.setAngle(shooterBed, Constants.ShooterBed.BED_INTAKE_PICKUP_ANGLE.getDegrees()) // Bed
            );
            // @formatter:on
        }
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
        int totalLEDs      = NUM_LEDS;
        
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
    // public static Command LEDFillDown(LED led, Color color)
    // {
    // // @formatter.off
    // int totalLEDs = NUM_LEDS;
    // Command[] commandSequence = new Command[totalLEDs];

    // return Commands.sequence(Commands.runOnce(() ->
    // {
    // var initialPattern = led.getLEDs();

    // for (int i = totalLEDs; i >= 0; i--)
    // {
    // Color[] newPattern = new Color[totalLEDs];
    // Arrays.fill(newPattern, i, totalLEDs, color);
    // System.arraycopy(initialPattern, 0, newPattern, 0, i);
    // commandSequence[totalLEDs - i] = LEDCommands.setFrame(led, newPattern);
    // }
    // }), Commands.sequence(commandSequence));

    // // Commands.sequence(LEDCommands.setFrame(led, IntStream.range(0,
    // // NUM_LEDS).mapToObj(i -> Intstream.range(0, NUM_LEDS).mapToObj(j -> ))))
    // // @formatter.on
    // }

    public static Command LEDSetSolidColor(LED led, Color color)
    {
        return LEDCommands.setFrame(led, IntStream.range(0, NUM_LEDS).mapToObj(i -> color).toArray(Color[]::new)).ignoringDisable(true);
    }

    public static Command LEDAutonomous(LED led)
    {
        Color color;

        switch (LoggedDriverStation.getDSData().allianceStation)
        {
            case DriverStationJNI.kRed1AllianceStation:
            case DriverStationJNI.kRed2AllianceStation:
            case DriverStationJNI.kRed3AllianceStation:
                color = RED;
                break;
            case DriverStationJNI.kBlue1AllianceStation:
            case DriverStationJNI.kBlue2AllianceStation:
            case DriverStationJNI.kBlue3AllianceStation:
                color = BLUE;
                break;
            default:
                color = ORANGE;
        }

        return CompositeCommands.LEDSetSolidColor(led, color);
    }

    public static Command LEDPulseColor(LED led, Color color)
    {
        return Commands.repeatingSequence(
                IntStream.range(0, 32)
                        .mapToObj(
                                i -> LEDSetSolidColor(
                                        led, new Color((int)(255 * color.red * (((i < 16) ? i : 32 - i) / 16.0)), (int)(255 * color.green * (((i < 16) ? i : 32 - i) / 16.0)), (int)(255 * color.blue * (((i < 16) ? i : 32 - i) / 16.0)))
                                )
                        ).toArray(Command[]::new)
        ).ignoringDisable(true);
    }

    // public static Command LEDTeleop(LED led)
    // {
    // return CompositeCommands.LEDFillDown(led, ORANGE).andThen(() ->
    // led.switchDefaultCommand(CompositeCommands.LEDSetSolidColor(led, ORANGE)));
    // }

    public static Command LEDSetDefaultColor(LED led, Color color)
    {
        return Commands.runOnce(() -> led.switchDefaultCommand(CompositeCommands.LEDSetSolidColor(led, color))).ignoringDisable(true);
    }

    public static Command LEDPartyMode(LED led)
    {
        //@formatter:off
        return Commands.repeatingSequence(
            new ProxyCommand(() -> LEDCommands.setFrame(led, led.getRandomColoring(PINK, ORANGE, TEAL))), 
            Commands.waitSeconds(0.5)).ignoringDisable(true);
        //@formatter:on
    }
}
