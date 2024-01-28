// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.notepath;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

/** Add your docs here. */
public class NotepathIOSim implements NotepathIO
{
    private DCMotorSim _leftNotepathSim = new DCMotorSim(DCMotor.getNeo550(1), 6.75, 0.025);
    private DCMotorSim _rightNotepathSim = new DCMotorSim(DCMotor.getNeo550(1), 6.75, 0.025);

    private double _voltage;

    @Override
    public void updateInputs(NotepathInputs inputs)
    {
        _leftNotepathSim.update(Constants.General.LOOP_PERIOD_SECS);
        _rightNotepathSim.update(Constants.General.LOOP_PERIOD_SECS);
        inputs.notepathAppliedVolts = _voltage;
        inputs.notepathCurrentAmps = new double[]
        { Math.abs(_leftNotepathSim.getCurrentDrawAmps()) };

    }

    @Override
    public void setVoltage(double volts)
    {
        _voltage = volts;
        _leftNotepathSim.setInputVoltage(_voltage);
        _rightNotepathSim.setInputVoltage(-_voltage);
    }
}
