// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

/** Vision subsystem hardware interface. */
public interface VisionIO {
  /** The set of loggable inputs for the vision subsystem. */
  @AutoLog  
  public static class VisionIOInputs {
    public double captureTimestamp = 0.0;
    public double[] cornerX = new double[] {};
    public double[] cornerY = new double[] {};
    public boolean simpleValid = false;
    public double simpleAngle = 0.0;
    
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs) {}

  /** Enabled or disabled vision LEDs. */
  public default void setLeds(boolean enabled) {}

  /** Sets the pipeline number. */
  public default void setPipeline(int pipeline) {}
}
