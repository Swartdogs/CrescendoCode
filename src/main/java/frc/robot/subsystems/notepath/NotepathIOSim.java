<<<<<<< HEAD
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

=======
>>>>>>> 0a065c2374e5de5f0d0c226630af8d26241ab0ee
package frc.robot.subsystems.notepath;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
<<<<<<< HEAD

/** Add your docs here. */
public class NotepathIOSim implements NotepathIO
{
    private DCMotorSim _leftNotepathSim = new DCMotorSim(DCMotor.getNeo550(1), 6.75, 0.025);
    private DCMotorSim _rightNotepathSim = new DCMotorSim(DCMotor.getNeo550(1), 6.75, 0.025);

    private double _voltage;
=======
import frc.robot.Constants;

public class NotepathIOSim implements NotepathIO
{
    private DCMotorSim _leftNotepathSim = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.025);
    private DCMotorSim _rightNotepathSim = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.025);
    private double _notepathAppliedVolts = 0.0;
>>>>>>> 0a065c2374e5de5f0d0c226630af8d26241ab0ee

    @Override
    public void updateInputs(NotepathInputs inputs)
    {
<<<<<<< HEAD
        inputs.notepathAppliedVolts = _voltage;
        inputs.notepathCurrentAmps = new double[]{_leftNotepathSim.getCurrentDrawAmps()};
=======
        _leftNotepathSim.update(Constants.General.LOOP_PERIOD_SECS);
        _rightNotepathSim.update(Constants.General.LOOP_PERIOD_SECS);
        inputs.notepathAppliedVolts = _notepathAppliedVolts;
        inputs.notepathCurrentAmps = new double[]
        { Math.abs(_leftNotepathSim.getCurrentDrawAmps()) };

>>>>>>> 0a065c2374e5de5f0d0c226630af8d26241ab0ee
    }

    @Override
    public void setVoltage(double volts)
    {
<<<<<<< HEAD
        _voltage = volts;
        _leftNotepathSim.setInputVoltage(volts);
        _rightNotepathSim.setInputVoltage(volts);

        
    }

=======
        _notepathAppliedVolts = volts;
        _leftNotepathSim.setInputVoltage(_notepathAppliedVolts);
        _rightNotepathSim.setInputVoltage(-_notepathAppliedVolts);
    }
>>>>>>> 0a065c2374e5de5f0d0c226630af8d26241ab0ee
}
