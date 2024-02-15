package frc.robot.commands;

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

    public static Command intakePickup(Intake intake, Notepath notepath, ShooterBed shooterBed, LED led)
    {
        return Commands.parallel(IntakeCommands.startIntake(intake), NotepathCommands.intakePickup(notepath), ShooterBedCommands.setBedIntakePickupAngle(shooterBed)).andThen(Commands.waitUntil(() -> notepath.sensorTripped()))
                .finallyDo(interrupted ->
                {
                    intake.setIntakeOff();
                    notepath.setOff();
                    if (interrupted)
                    {
                        LEDBlinking(led, Constants.LED.ORANGE);
                        notepath.setHasNote(true);
                    }
                }).unless(() -> notepath.hasNote());
    }

    public static Command stopIntaking(Intake intake, Notepath notepath)
    {
        return Commands.parallel(IntakeCommands.stopIntake(intake), NotepathCommands.stopFeed(notepath));
    }

    public static Command shooterPickup(ShooterBed shooterBed, ShooterFlywheel shooterFlywheel, Notepath notepath, LED led)
    {
        return Commands.parallel(ShooterBedCommands.setBedShooterPickupAngle(shooterBed), ShooterFlywheelCommands.shooterFlywheelIntake(shooterFlywheel), NotepathCommands.reverseFeed(notepath))
                .andThen(Commands.waitUntil(() -> notepath.sensorTripped())).andThen(Commands.waitUntil(() -> !notepath.sensorTripped())).finallyDo(interrupted ->
                {
                    shooterFlywheel.stop();
                    notepath.setOff();
                    if (interrupted)
                    {
                        LEDBlinking(led, Constants.LED.ORANGE);
                        notepath.setHasNote(true);
                    }
                }).unless(() -> notepath.hasNote());
    }

    public static Command LEDFillUp(LED led, Color color)
    {
        var initialPattern = led.getLEDs();

        var _1  = initialPattern.get(1);
        var _2  = initialPattern.get(2);
        var _3  = initialPattern.get(3);
        var _4  = initialPattern.get(4);
        var _5  = initialPattern.get(5);
        var _6  = initialPattern.get(6);
        var _7  = initialPattern.get(7);
        var _8  = initialPattern.get(8);
        var _9  = initialPattern.get(9);
        var _10 = initialPattern.get(10);
        var _11 = initialPattern.get(11);
        var _12 = initialPattern.get(12);
        var _13 = initialPattern.get(13);
        var _14 = initialPattern.get(14);
        var _15 = initialPattern.get(15);
        var _16 = initialPattern.get(16);
        var _17 = initialPattern.get(17);
        var _18 = initialPattern.get(18);
        var _19 = initialPattern.get(19);
        var _20 = initialPattern.get(20);

        return Commands.sequence(
                LEDCommands.setFrame(led, new Color[] { color, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20 }),
                LEDCommands.setFrame(led, new Color[] { color, color, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20 }),
                LEDCommands.setFrame(led, new Color[] { color, color, color, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20 }),
                LEDCommands.setFrame(led, new Color[] { color, color, color, color, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20 }),
                LEDCommands.setFrame(led, new Color[] { color, color, color, color, color, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20 }),
                LEDCommands.setFrame(led, new Color[] { color, color, color, color, color, color, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20 }),
                LEDCommands.setFrame(led, new Color[] { color, color, color, color, color, color, color, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20 }),
                LEDCommands.setFrame(led, new Color[] { color, color, color, color, color, color, color, color, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20 }),
                LEDCommands.setFrame(led, new Color[] { color, color, color, color, color, color, color, color, color, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20 }),
                LEDCommands.setFrame(led, new Color[] { color, color, color, color, color, color, color, color, color, color, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20 }),
                LEDCommands.setFrame(led, new Color[] { color, color, color, color, color, color, color, color, color, color, color, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20 }),
                LEDCommands.setFrame(led, new Color[] { color, color, color, color, color, color, color, color, color, color, color, color, _12, _13, _14, _15, _16, _17, _18, _19, _20 }),
                LEDCommands.setFrame(led, new Color[] { color, color, color, color, color, color, color, color, color, color, color, color, color, _13, _14, _15, _16, _17, _18, _19, _20 }),
                LEDCommands.setFrame(led, new Color[] { color, color, color, color, color, color, color, color, color, color, color, color, color, color, _14, _15, _16, _17, _18, _19, _20 }),
                LEDCommands.setFrame(led, new Color[] { color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, _15, _16, _17, _18, _19, _20 }),
                LEDCommands.setFrame(led, new Color[] { color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, _16, _17, _18, _19, _20 }),
                LEDCommands.setFrame(led, new Color[] { color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, _17, _18, _19, _20 }),
                LEDCommands.setFrame(led, new Color[] { color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, _18, _19, _20 }),
                LEDCommands.setFrame(led, new Color[] { color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, _19, _20 }),
                LEDCommands.setFrame(led, new Color[] { color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, _20 }),
                LEDCommands.setFrame(led, new Color[] { color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color })
        );
    }

    public static Command LEDFillDown(LED led, Color color)
    {
        var initialPattern = led.getLEDs();

        var _0  = initialPattern.get(0);
        var _1  = initialPattern.get(1);
        var _2  = initialPattern.get(2);
        var _3  = initialPattern.get(3);
        var _4  = initialPattern.get(4);
        var _5  = initialPattern.get(5);
        var _6  = initialPattern.get(6);
        var _7  = initialPattern.get(7);
        var _8  = initialPattern.get(8);
        var _9  = initialPattern.get(9);
        var _10 = initialPattern.get(10);
        var _11 = initialPattern.get(11);
        var _12 = initialPattern.get(12);
        var _13 = initialPattern.get(13);
        var _14 = initialPattern.get(14);
        var _15 = initialPattern.get(15);
        var _16 = initialPattern.get(16);
        var _17 = initialPattern.get(17);
        var _18 = initialPattern.get(18);
        var _19 = initialPattern.get(19);

        return Commands.sequence(
                LEDCommands.setFrame(led, new Color[] { _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, color }),
                LEDCommands.setFrame(led, new Color[] { _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, color, color }),
                LEDCommands.setFrame(led, new Color[] { _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, color, color, color }),
                LEDCommands.setFrame(led, new Color[] { _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, color, color, color, color }),
                LEDCommands.setFrame(led, new Color[] { _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, color, color, color, color, color }),
                LEDCommands.setFrame(led, new Color[] { _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, color, color, color, color, color, color }),
                LEDCommands.setFrame(led, new Color[] { _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, color, color, color, color, color, color, color }),
                LEDCommands.setFrame(led, new Color[] { _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, color, color, color, color, color, color, color, color }),
                LEDCommands.setFrame(led, new Color[] { _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, color, color, color, color, color, color, color, color, color }),
                LEDCommands.setFrame(led, new Color[] { _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, color, color, color, color, color, color, color, color, color, color }),
                LEDCommands.setFrame(led, new Color[] { _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, color, color, color, color, color, color, color, color, color, color, color }),
                LEDCommands.setFrame(led, new Color[] { _0, _1, _2, _3, _4, _5, _6, _7, _8, color, color, color, color, color, color, color, color, color, color, color, color }),
                LEDCommands.setFrame(led, new Color[] { _0, _1, _2, _3, _4, _5, _6, _7, color, color, color, color, color, color, color, color, color, color, color, color, color }),
                LEDCommands.setFrame(led, new Color[] { _0, _1, _2, _3, _4, _5, _6, color, color, color, color, color, color, color, color, color, color, color, color, color, color }),
                LEDCommands.setFrame(led, new Color[] { _0, _1, _2, _3, _4, _5, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color }),
                LEDCommands.setFrame(led, new Color[] { _0, _1, _2, _3, _4, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color }),
                LEDCommands.setFrame(led, new Color[] { _0, _1, _2, _3, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color }),
                LEDCommands.setFrame(led, new Color[] { _0, _1, _2, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color }),
                LEDCommands.setFrame(led, new Color[] { _0, _1, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color }),
                LEDCommands.setFrame(led, new Color[] { _0, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color }),
                LEDCommands.setFrame(led, new Color[] { color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color })
        );
    }

    public static Command LEDBlinking(LED led, Color color)
    {
        return Commands.repeatingSequence(
                LEDCommands.setFrame(led, new Color[] { color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color }), Commands.waitSeconds(1),
                LEDCommands.setFrame(led, new Color[] { null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null }), Commands.waitSeconds(1)
        );
    }

    public static Command LEDMovingStrip(LED led, Color color)
    {
        var initialPattern = led.getLEDs();

        var _0  = initialPattern.get(0);
        var _1  = initialPattern.get(1);
        var _2  = initialPattern.get(2);
        var _3  = initialPattern.get(3);
        var _4  = initialPattern.get(4);
        var _5  = initialPattern.get(5);
        var _6  = initialPattern.get(6);
        var _7  = initialPattern.get(7);
        var _8  = initialPattern.get(8);
        var _9  = initialPattern.get(9);
        var _10 = initialPattern.get(10);
        var _11 = initialPattern.get(11);
        var _12 = initialPattern.get(12);
        var _13 = initialPattern.get(13);
        var _14 = initialPattern.get(14);
        var _15 = initialPattern.get(15);
        var _16 = initialPattern.get(16);
        var _17 = initialPattern.get(17);
        var _18 = initialPattern.get(18);
        var _19 = initialPattern.get(19);
        var _20 = initialPattern.get(20);

        return Commands.repeatingSequence(
                LEDCommands.setFrame(led, new Color[] { color, color, color, color, color, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20 }),
                LEDCommands.setFrame(led, new Color[] { _0, color, color, color, color, color, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20 }),
                LEDCommands.setFrame(led, new Color[] { _0, _1, color, color, color, color, color, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20 }),
                LEDCommands.setFrame(led, new Color[] { _0, _1, _2, color, color, color, color, color, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20 }),
                LEDCommands.setFrame(led, new Color[] { _0, _1, _2, _3, color, color, color, color, color, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20 }),
                LEDCommands.setFrame(led, new Color[] { _0, _1, _2, _3, _4, color, color, color, color, color, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20 }),
                LEDCommands.setFrame(led, new Color[] { _0, _1, _2, _3, _4, _5, color, color, color, color, color, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20 }),
                LEDCommands.setFrame(led, new Color[] { _0, _1, _2, _3, _4, _5, _6, color, color, color, color, color, _12, _13, _14, _15, _16, _17, _18, _19, _20 }),
                LEDCommands.setFrame(led, new Color[] { _0, _1, _2, _3, _4, _5, _6, _7, color, color, color, color, color, _13, _14, _15, _16, _17, _18, _19, _20 }),
                LEDCommands.setFrame(led, new Color[] { _0, _1, _2, _3, _4, _5, _6, _7, _8, color, color, color, color, color, _14, _15, _16, _17, _18, _19, _20 }),
                LEDCommands.setFrame(led, new Color[] { _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, color, color, color, color, color, _15, _16, _17, _18, _19, _20 }),
                LEDCommands.setFrame(led, new Color[] { _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, color, color, color, color, color, _16, _17, _18, _19, _20 }),
                LEDCommands.setFrame(led, new Color[] { _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, color, color, color, color, color, _17, _18, _19, _20 }),
                LEDCommands.setFrame(led, new Color[] { _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, color, color, color, color, color, _18, _19, _20 }),
                LEDCommands.setFrame(led, new Color[] { _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, color, color, color, color, color, _19, _20 }),
                LEDCommands.setFrame(led, new Color[] { _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, color, color, color, color, color, _20 }),
                LEDCommands.setFrame(led, new Color[] { _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, color, color, color, color, color }),
                LEDCommands.setFrame(led, new Color[] { color, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, color, color, color, color }),
                LEDCommands.setFrame(led, new Color[] { color, color, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, color, color, color }),
                LEDCommands.setFrame(led, new Color[] { color, color, color, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, color, color }),
                LEDCommands.setFrame(led, new Color[] { color, color, color, color, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, color })
        );
    }

    public static Command LEDFlashSolidColor(LED led, Color color)
    {
        var initialPattern = led.getLEDs();

        var _0  = initialPattern.get(0);
        var _1  = initialPattern.get(1);
        var _2  = initialPattern.get(2);
        var _3  = initialPattern.get(3);
        var _4  = initialPattern.get(4);
        var _5  = initialPattern.get(5);
        var _6  = initialPattern.get(6);
        var _7  = initialPattern.get(7);
        var _8  = initialPattern.get(8);
        var _9  = initialPattern.get(9);
        var _10 = initialPattern.get(10);
        var _11 = initialPattern.get(11);
        var _12 = initialPattern.get(12);
        var _13 = initialPattern.get(13);
        var _14 = initialPattern.get(14);
        var _15 = initialPattern.get(15);
        var _16 = initialPattern.get(16);
        var _17 = initialPattern.get(17);
        var _18 = initialPattern.get(18);
        var _19 = initialPattern.get(19);
        var _20 = initialPattern.get(20);

        return Commands.sequence(
                LEDCommands.setFrame(led, new Color[] { color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color, color }), Commands.waitSeconds(3),
                LEDCommands.setFrame(led, new Color[] { _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20 })
        );
    }
}
