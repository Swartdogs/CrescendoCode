package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class ShooterFlywheelIOSim implements ShooterFlywheelIO
{
    private DCMotorSim _upperFlywheelSim = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.025);
    private DCMotorSim _lowerFlywheelSim = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.025);

    private double _upperAppliedVolts = 0;
    private double _lowerAppliedVolts = 0;

    @Override
    public void updateInputs(ShooterFlywheelIOInputs inputs)
    {
        _upperFlywheelSim.update(Constants.General.LOOP_PERIOD_SECS);
        _lowerFlywheelSim.update(Constants.General.LOOP_PERIOD_SECS);
        
        inputs.upperFlywheelAppliedVolts = _upperAppliedVolts;
        inputs.upperFlywheelCurrentAmps = new double[]
        { Math.abs(_upperFlywheelSim.getCurrentDrawAmps()) };
        
        inputs.lowerFlywheelAppliedVolts = _lowerAppliedVolts;
        inputs.lowerFlywheelCurrentAmps = new double[]
        { Math.abs(_lowerFlywheelSim.getCurrentDrawAmps()) };
    }

    @Override
    public void setUpperVoltage(double upperVolts)
    {
        _upperFlywheelSim.setInputVoltage(upperVolts);
    }

    @Override
    public void setLowerVoltage(double lowerVolts)
    {
        _lowerFlywheelSim.setInputVoltage(lowerVolts);
    }
}
