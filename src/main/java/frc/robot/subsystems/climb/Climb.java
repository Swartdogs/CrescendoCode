// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase
{
    private final ClimbIO                 _io;
    private final ClimbIOInputsAutoLogged _inputs = new ClimbIOInputsAutoLogged();
    private final PIDController           _climbFeedback;

    private Double _climbSetpoint = null;

    public Climb(ClimbIO io)
    {
        _io            = io;
        _climbFeedback = new PIDController(0, 0, 0); // TODO: tune values
    }

    @Override
    public void periodic()
    {
        _io.updateInputs(_inputs);
        Logger.processInputs("Climb", _inputs);
    }
}
