package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO
{
    private final DCMotorSim _intakeSim = new DCMotorSim(DCMotor.getNeo550(1), 6.75, 0.025);
    private double           _voltage;

    @Override
    public void updateInputs(IntakeIOInputs inputs)
    {
        _intakeSim.update(Constants.General.LOOP_PERIOD_SECS);

        inputs.motorVolts   = _voltage;
        inputs.motorCurrent = new double[] { _intakeSim.getCurrentDrawAmps() };
    }

    @Override
    public void setVoltage(double volts)
    {
        _voltage = volts;
        _intakeSim.setInputVoltage(volts);
    }
}
