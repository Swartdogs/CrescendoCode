// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.gyro;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI.Port;

public class GyroIONavX2 implements GyroIO
{
    private final AHRS _gyro = new AHRS(Port.kMXP);

    public GyroIONavX2()
    {
        _gyro.reset();
    }

    public void updateInputs(GyroIOInputs inputs)
    {
        inputs.yawPosition          = Rotation2d.fromDegrees(-_gyro.getYaw());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(-_gyro.getRate());
        inputs.rollPosition         = Rotation2d.fromDegrees(_gyro.getRoll());
    }
}
