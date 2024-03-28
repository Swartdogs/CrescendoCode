// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.leds;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.util.Color;

public interface LEDIO
{
    public default void applyAnimationFrame(ArrayList<Color> pattern)
    {

    }

    public default ArrayList<Color> getLEDs()
    {
        return null;
    }
}