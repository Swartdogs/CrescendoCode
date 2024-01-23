// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

public class CmdClimbWithJoystick extends Command
{
    private Climb _climb = Climb.getInstance();
 
    private DoubleSupplier _yLeftSupplier;
    private DoubleSupplier _yRightSupplier;

    public CmdClimbWithJoystick(Climb climb, DoubleSupplier yLeftSupplier, DoubleSupplier yRightSupplier)
    {
        addRequirements(_climb);
        _yLeftSupplier = yLeftSupplier;
        _yRightSupplier = yRightSupplier;
    }

    @Override
    public void execute()
    {
        _climb.setVoltageLeft(_yLeftSupplier.getAsDouble());
        _climb.setVoltageRight(_yRightSupplier.getAsDouble());
        
        // TODO add scaling 
    }

}
