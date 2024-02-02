// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

public class CmdAveragePoint extends Command
{
    private final Climb _climb;

    private final DoubleSupplier _ySupplier;

    private final double _averagePoint;
     
    public CmdAveragePoint(Climb climb, DoubleSupplier ySupplier, double averagePoint)
    {
        _climb        = climb;
        _ySupplier    = ySupplier;
        _averagePoint = averagePoint;

        addRequirements(_climb);
    }

    @Override
    public void initialize()
    {
        _climb.setHeight(_averagePoint);
    }

    @Override
    public void execute()
    {
        _climb.setVoltageLeft(_ySupplier.getAsDouble() * 12);        
    }

    @Override
    public void end(boolean interrupted)
    {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
