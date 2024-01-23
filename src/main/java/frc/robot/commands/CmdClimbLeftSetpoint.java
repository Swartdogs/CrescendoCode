// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class CmdClimbLeftSetpoint extends Command
{

    private Double _setpoint;

    public CmdClimbLeftSetpoint(Double setpoint)
    {
        _setpoint = setpoint;
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled. Sets the setpoint
    @Override
    public void initialize()
    {}

    // Returns true when the command should end. Check if we're at the setpoint
    @Override
    public boolean isFinished()
    {

        return false;
    }
}
