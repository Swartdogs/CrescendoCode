package frc.robot.subsystems.notepath;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class NotepathIOSim implements NotepathIO
{
    private final DCMotorSim   _leaderMotorSim   = new DCMotorSim(DCMotor.getNeo550(1), 6.75, 0.025);
    private final DCMotorSim   _followerMotorSim = new DCMotorSim(DCMotor.getNeo550(1), 6.75, 0.025);
    private final DigitalInput _lightSensorSim   = new DigitalInput(Constants.DIO.NOTE_SENSOR);
    private double             _leaderVolts;
    private double             _followerVolts;

    @Override
    public void updateInputs(NotepathInputs inputs)
    {
        _leaderMotorSim.update(Constants.General.LOOP_PERIOD_SECS);
        _followerMotorSim.update(Constants.General.LOOP_PERIOD_SECS);

        inputs.leaderVolts   = _leaderVolts;
        inputs.leaderCurrent = new double[] { Math.abs(_leaderMotorSim.getCurrentDrawAmps()) };

        inputs.followerVolts   = _followerVolts;
        inputs.followerCurrent = new double[] { Math.abs(_followerMotorSim.getCurrentDrawAmps()) };
        inputs.sensorTripped   = !_lightSensorSim.get();
    }

    @Override
    public void setVolts(double volts)
    {
        _leaderVolts   = volts;
        _followerVolts = volts;
        _leaderMotorSim.setInputVoltage(_leaderVolts);
        _followerMotorSim.setInputVoltage(_followerVolts);
    }
}
