// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.notepath.Notepath;

public class CmdNotepathStopNotepath extends Command
{
    private Notepath _notepath;

    public CmdNotepathStopNotepath(Notepath notepath)
    {
        _notepath = notepath;

        addRequirements(_notepath);
    }

    @Override
    public void initialize()
    {
        _notepath.setOff();

    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}
