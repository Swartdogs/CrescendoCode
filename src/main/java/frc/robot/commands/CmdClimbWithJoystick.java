// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;
import java.util.function.DoubleSupplier;

public class CmdClimbWithJoystick extends Command
{
    private final Climb          _climb;
    private final DoubleSupplier _yLeftSupplier;
    private final DoubleSupplier _yRightSupplier;

    public CmdClimbWithJoystick(Climb climb, DoubleSupplier yLeftSupplier, DoubleSupplier yRightSupplier)
    {
        _climb = climb;

        _yLeftSupplier  = yLeftSupplier;
        _yRightSupplier = yRightSupplier;

        addRequirements(_climb);
    }

    @Override
    public void execute()
    {
        _climb.setVoltageLeft(_yLeftSupplier.getAsDouble() * 12);
        _climb.setVoltageRight(_yRightSupplier.getAsDouble() * 12); // TODO add scaling
    }

    @Override
    public void end(boolean interrupted)
    {
        _climb.stop();
    }
}
