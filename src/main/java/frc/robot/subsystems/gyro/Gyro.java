// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.gyro;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase
{
    private final GyroIO                 _io;
    private final GyroIOInputsAutoLogged _inputs = new GyroIOInputsAutoLogged();

    public Gyro(GyroIO io)
    {
        _io = io;
    }

    @Override
    public void periodic()
    {
        _io.updateInputs(_inputs);
        Logger.processInputs("Gyro", _inputs);
    }

    public Rotation2d getYawPosition()
    {
        return _inputs.yawPosition;
    }

    public double getYawVelocityRadPerSec()
    {
        return _inputs.yawVelocityRadPerSec;
    }

    public Rotation2d getRollPosition()
    {
        return _inputs.rollPosition;
    }
}
