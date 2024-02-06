// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO
{
    @AutoLog
    public static class ClimbIOInputs
    {
        public double extensionLeft     = 0.0;
        public double extensionRight    = 0.0;
        public double appliedVoltsLeft  = 0.0;
        public double appliedVoltsRight = 0.0;
    }

    public default void updateInputs(ClimbIOInputs inputs)
    {
    }

    public default void setVoltageLeft(double volts)
    {
    }

    public default void setVoltageRight(double volts)
    {
    }

    public default void setLeftAngleOffset(double leftAbsoluteEncoderOffset)
    {
    }

    public default void setRightAngleOffset(double rightAbsoluteEncoderOffset)
    {
    }
}
