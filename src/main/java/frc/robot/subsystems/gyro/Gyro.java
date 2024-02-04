// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gyro;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.GyroIOInputsAutoLogged;

public class Gyro extends SubsystemBase
{
    private final GyroIO                 _gyroIO;
    private final GyroIOInputsAutoLogged _gyroInputs = new GyroIOInputsAutoLogged();

    public Gyro(GyroIO gyroIO)
    {
        _gyroIO = gyroIO;
    }

    public void periodic()
    {
        _gyroIO.updateInputs(_gyroInputs);
        Logger.processInputs("Gyro", _gyroInputs);
    }
}
