// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.gyro;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO
{
    @AutoLog
    public static class GyroIOInputs
    {
        public Rotation2d yawPosition          = new Rotation2d();
        public double     yawVelocityRadPerSec = 0.0;
        public Rotation2d rollPosition         = new Rotation2d();
    }

    public default void updateInputs(GyroIOInputsAutoLogged inputs)
    {
    }
}
