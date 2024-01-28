// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.notepath;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class NotepathIOSim implements NotepathIO
{
    private DCMotorSim _leftNotepathSim = new DCMotorSim(DCMotor.getNeo550(1), 6.75, 0.025);
    private DCMotorSim _rightNotepathSim = new DCMotorSim(DCMotor.getNeo550(1), 6.75, 0.025);

    private double _voltage;

    @Override
    public void updateInputs(NotepathInputs inputs)
    {
        inputs.notepathAppliedVolts = _voltage;
        inputs.notepathCurrentAmps = new double[]{_leftNotepathSim.getCurrentDrawAmps()};
    }

    @Override
    public void setVoltage(double volts)
    {
        _voltage = volts;
        _leftNotepathSim.setInputVoltage(volts);
        _rightNotepathSim.setInputVoltage(volts);

        
    }

}
