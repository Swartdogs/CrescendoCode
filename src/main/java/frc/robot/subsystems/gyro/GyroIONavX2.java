// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gyro;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI.Port;
import frc.robot.subsystems.drive.GyroIO;

public class GyroIONavX2 implements GyroIO
{
    private final AHRS _navx = new AHRS(Port.kMXP);

    public GyroIONavX2()
    {
        _navx.reset();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs)
    {
        
    }
}
