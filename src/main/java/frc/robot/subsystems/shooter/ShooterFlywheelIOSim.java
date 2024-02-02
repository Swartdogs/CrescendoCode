package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class ShooterFlywheelIOSim implements ShooterFlywheelIO
{
    private final double UPPER_MOTOR_MAX_VOLTAGE;
    private final double LOWER_MOTOR_MAX_VOLTAGE;

    private DCMotor    _upperMotor        = DCMotor.getNEO(1);
    private DCMotor    _lowerMotor        = DCMotor.getNEO(1);
    private DCMotorSim _upperFlywheelSim  = new DCMotorSim(_upperMotor, 6.75, 0.025);
    private DCMotorSim _lowerFlywheelSim  = new DCMotorSim(_lowerMotor, 6.75, 0.025);

    private double     _upperAppliedVolts = 0;
    private double     _lowerAppliedVolts = 0;

    public ShooterFlywheelIOSim()
    {
        // getVoltage() calculates voltage needed for a given torque at a given speed. Torque at max speed is 0
        UPPER_MOTOR_MAX_VOLTAGE = _upperMotor.getVoltage(0, _upperMotor.freeSpeedRadPerSec);
        LOWER_MOTOR_MAX_VOLTAGE = _lowerMotor.getVoltage(0, _lowerMotor.freeSpeedRadPerSec);
    }

    @Override
    public void updateInputs(ShooterFlywheelIOInputs inputs)
    {
        _upperFlywheelSim.update(Constants.General.LOOP_PERIOD_SECS);
        _lowerFlywheelSim.update(Constants.General.LOOP_PERIOD_SECS);

        inputs.upperFlywheelVelocity     = _upperFlywheelSim.getAngularVelocityRPM();
        inputs.upperFlywheelAppliedVolts = _upperAppliedVolts;
        inputs.upperFlywheelCurrentAmps  = new double[] { Math.abs(_upperFlywheelSim.getCurrentDrawAmps()) };

        inputs.lowerFlywheelVelocity     = _lowerFlywheelSim.getAngularVelocityRPM();
        inputs.lowerFlywheelAppliedVolts = _lowerAppliedVolts;
        inputs.lowerFlywheelCurrentAmps  = new double[] { Math.abs(_lowerFlywheelSim.getCurrentDrawAmps()) };
    }

    @Override
    public void setUpperVelocity(double velocity)
    {
        _upperAppliedVolts = UPPER_MOTOR_MAX_VOLTAGE * velocity / Units.radiansPerSecondToRotationsPerMinute(_upperMotor.freeSpeedRadPerSec);
        _upperFlywheelSim.setInputVoltage(_upperAppliedVolts);
    }
    
    @Override
    public void setLowerVelocity(double velocity)
    {
        _lowerAppliedVolts = LOWER_MOTOR_MAX_VOLTAGE * velocity / Units.radiansPerSecondToRotationsPerMinute(_lowerMotor.freeSpeedRadPerSec);
        _lowerFlywheelSim.setInputVoltage(_lowerAppliedVolts);
    }

    @Override
    public void setUpperVoltage(double upperVolts)
    {
        _upperFlywheelSim.setInputVoltage(upperVolts);
        _upperAppliedVolts = upperVolts;
    }

    @Override
    public void setLowerVoltage(double lowerVolts)
    {
        _lowerFlywheelSim.setInputVoltage(lowerVolts);
        _lowerAppliedVolts = lowerVolts;
    }
}
