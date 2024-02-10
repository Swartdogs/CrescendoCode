// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climb.ClimbIO;

public class LED extends SubsystemBase
{
    private final LEDIO                 _io;
    private final LEDIOInputsAutoLogged _inputs = new LEDInputsAutoLogged();

    public LED()
    {

    }

    @Override
    public void periodic()
    {

    }
}
