// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.climb.Climb;

public class CmdClimbDriveManual extends Command
{
    private final Climb          _climb;
    private final DoubleSupplier _yLeftSupplier;
    private final DoubleSupplier _yRightSupplier;

    public CmdClimbDriveManual(Climb climb, DoubleSupplier yLeftSupplier, DoubleSupplier yRightSupplier)
    {
        _climb = climb;

        _yLeftSupplier  = yLeftSupplier;
        _yRightSupplier = yRightSupplier;

        addRequirements(_climb);
    }

    @Override
    public void initialize()
    {
        _climb.setVoltageLeft(_yLeftSupplier.getAsDouble() * Constants.General.MOTOR_VOLTAGE);
        _climb.setVoltageRight(_yRightSupplier.getAsDouble() * Constants.General.MOTOR_VOLTAGE); // TODO add scaling
    }

    @Override
    public void end(boolean interrupted)
    {
        _climb.stop();
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
