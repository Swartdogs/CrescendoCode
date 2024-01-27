package frc.robot.subsystems.notepath;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class NotepathIOSim implements NotepathIO
{
    private DCMotorSim _leftNotepathSim = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.025);
    private DCMotorSim _rightNotepathSim = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.025);
    private double _notepathAppliedVolts = 0.0;

    @Override
    public void updateInputs(NotepathInputs inputs)
    {
        _leftNotepathSim.update(Constants.General.LOOP_PERIOD_SECS);
        _rightNotepathSim.update(Constants.General.LOOP_PERIOD_SECS);
        inputs.notepathAppliedVolts = _notepathAppliedVolts;
        inputs.notepathCurrentAmps = new double[]
        { Math.abs(_leftNotepathSim.getCurrentDrawAmps()) };

    }

    @Override
    public void setVoltage(double volts)
    {
        _notepathAppliedVolts = volts;
        _leftNotepathSim.setInputVoltage(_notepathAppliedVolts);
        _rightNotepathSim.setInputVoltage(-_notepathAppliedVolts);
    }
}
