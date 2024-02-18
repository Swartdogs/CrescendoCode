package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import frc.robot.Constants;

public class ShooterFlywheelIOSim implements ShooterFlywheelIO
{
    private final double     UPPER_MOTOR_MAX_VOLTAGE;
    private final double     LOWER_MOTOR_MAX_VOLTAGE;
    private final DCMotor    _upperMotor        = DCMotor.getNEO(1);
    private final DCMotor    _lowerMotor        = DCMotor.getNEO(1);
    private final DCMotorSim _upperMotorSim     = new DCMotorSim(_upperMotor, 1, 0.001);
    private final DCMotorSim _lowerMotorSim     = new DCMotorSim(_lowerMotor, 1, 0.001);
    private double           _upperAppliedVolts = 0;
    private double           _lowerAppliedVolts = 0;

    public ShooterFlywheelIOSim()
    {
        // getVoltage() calculates voltage needed for a given torque at a given speed.
        // Torque at max speed is 0
        UPPER_MOTOR_MAX_VOLTAGE = _upperMotor.getVoltage(0, _upperMotor.freeSpeedRadPerSec);
        LOWER_MOTOR_MAX_VOLTAGE = _lowerMotor.getVoltage(0, _lowerMotor.freeSpeedRadPerSec);
    }

    @Override
    public void updateInputs(ShooterFlywheelIOInputs inputs)
    {
        _upperMotorSim.update(Constants.General.LOOP_PERIOD_SECS);
        _lowerMotorSim.update(Constants.General.LOOP_PERIOD_SECS);

        inputs.upperVelocity = _upperMotorSim.getAngularVelocityRPM();
        inputs.upperVolts    = _upperAppliedVolts;
        inputs.upperCurrent  = new double[] { Math.abs(_upperMotorSim.getCurrentDrawAmps()) };

        inputs.lowerVelocity = _lowerMotorSim.getAngularVelocityRPM();
        inputs.lowerVolts    = _lowerAppliedVolts;
        inputs.lowerCurrent  = new double[] { Math.abs(_lowerMotorSim.getCurrentDrawAmps()) };
    }

    @Override
    public void setUpperVelocity(double velocity)
    {
        _upperAppliedVolts = UPPER_MOTOR_MAX_VOLTAGE * velocity / Units.radiansPerSecondToRotationsPerMinute(_upperMotor.freeSpeedRadPerSec);
        _upperMotorSim.setInputVoltage(_upperAppliedVolts);
    }

    @Override
    public void setLowerVelocity(double velocity)
    {
        _lowerAppliedVolts = LOWER_MOTOR_MAX_VOLTAGE * velocity / Units.radiansPerSecondToRotationsPerMinute(_lowerMotor.freeSpeedRadPerSec);
        _lowerMotorSim.setInputVoltage(_lowerAppliedVolts);
    }

    @Override
    public void setUpperVolts(double volts)
    {
        _upperMotorSim.setInputVoltage(volts);
        _upperAppliedVolts = volts;
    }

    @Override
    public void setLowerVolts(double volts)
    {
        _lowerMotorSim.setInputVoltage(volts);
        _lowerAppliedVolts = volts;
    }
}
