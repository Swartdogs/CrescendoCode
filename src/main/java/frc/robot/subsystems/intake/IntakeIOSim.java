package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO
{
    private final DCMotorSim _motorSim = new DCMotorSim(DCMotor.getNeo550(1), 6.75, 0.025);
    private double           _volts;

    @Override
    public void updateInputs(IntakeIOInputs inputs)
    {
        _motorSim.update(Constants.General.LOOP_PERIOD_SECS);

        inputs.motorVolts   = _volts;
        inputs.motorCurrent = new double[] { _motorSim.getCurrentDrawAmps() };
    }

    @Override
    public void setVolts(double volts)
    {
        _volts = volts;
        _motorSim.setInputVoltage(volts);
    }
}
