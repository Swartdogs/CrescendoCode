package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.leds.LED;

public final class LEDCommands
{
    private LEDCommands()
    {
    }

    public static Command setFrame(LED led, Color[] colorList)
    {
        return Commands.runOnce(() -> led.applyAnimationFrame(colorList), led).ignoringDisable(true);
    }
}
