// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

public class CmdClimbRightSetpoint extends Command {
  private final Climb _climb;
  private final double _setpoint;

  public CmdClimbRightSetpoint(Climb climb, double setpoint) {
    _climb = climb;
    _setpoint = setpoint;
  }

  @Override
  public void initialize() {
    _climb.setSetpointRight(_setpoint);
  }

  @Override
  public boolean isFinished() {
    return _climb.isAtRightSetpoint();
  }
}
