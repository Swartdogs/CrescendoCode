// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

public class DeferredInstantCommand extends Command
{
    private final Supplier<Command> _supplier;

    public DeferredInstantCommand(Supplier<Command> supplier)
    {
        _supplier = supplier;
    }

    @Override
    public void initialize()
    {
        _supplier.get().schedule();
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}
