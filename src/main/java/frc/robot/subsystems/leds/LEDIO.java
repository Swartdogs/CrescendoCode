// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.util.Color;

public interface LEDIO
{
    @AutoLog
    public static class LEDIOInputs
    {
        public double r   = 0.0;
        public double g   = 0.0;
        public double b   = 0.0;
        public String hex = "";
    }

    public default void updateInputs(LEDIOInputs inputs)
    {
    }

    public default void applyAnimationFrame(Color[] pattern)
    {

    }

    public default Color[] getLEDs()
    {
        return null;
    }
}
