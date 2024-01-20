// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ClimbIO 
{
    @AutoLog
    public static class ClimbIOInputs
    {
        public double  measuredExtension = 0.0;   // from potentimeter
        public boolean lockState         = false; // solenoid
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ClimbIOInputs inputs){}

    /** Run the climb motor at the specified voltage. */
    public default void setVoltage(double volts){}

    /** Enable or disable brake mode on the drive motor. */
    public default void setLockState(boolean enable){}
}
