package frc.robot.subsystems.Intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO
{
    private DCMotorSim _intakeSim = new DCMotorSim(DCMotor.getNeo550(1), 6.75, 0.025);
    private double     _voltage;

    @Override
    public void updateInputs(IntakeIOInputs inputs)
    {
        _intakeSim.update(Constants.General.LOOP_PERIOD_SECS);

        inputs.appliedVolts = _voltage;
        inputs.currentAmps  = new double[] { _intakeSim.getCurrentDrawAmps() };
    }

    @Override
    public void setVoltage(double volts)
    {
        _voltage = volts;
        _intakeSim.setInputVoltage(volts);
    }
}
