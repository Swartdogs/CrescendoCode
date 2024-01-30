// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.notepath.Notepath;

public class CmdNotepathStartFeed extends Command
{
    private Notepath _notepath;

    public CmdNotepathStartFeed(Notepath notepath)
    {// Use addRequirements() here to declare subsystem dependencies.
        _notepath = notepath;
        
        addRequirements(_notepath);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        _notepath.setFeedOn();

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        _notepath.setOff();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
