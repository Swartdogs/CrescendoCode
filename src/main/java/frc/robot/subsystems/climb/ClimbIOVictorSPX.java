package frc.robot.subsystems.climb;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import frc.robot.Constants;

public class ClimbIOVictorSPX implements ClimbIO
{
    private final VictorSPX           _leftArmMotor;
    private final VictorSPX           _rightArmMotor;
    private final AnalogPotentiometer _leftPot;
    private final AnalogPotentiometer _rightPot;
    private double                    _leftOffset  = Constants.Climb.LEFT_ZERO_OFFSET;
    private double                    _rightOffset = Constants.Climb.RIGHT_ZERO_OFFSET;

    public ClimbIOVictorSPX()
    {
        _leftArmMotor  = new VictorSPX(Constants.CAN.CLIMB_LEFT);
        _rightArmMotor = new VictorSPX(Constants.CAN.CLIMB_RIGHT);

        _leftPot  = new AnalogPotentiometer(Constants.AIO.CLIMB_LEFT_SENSOR, Constants.Climb.SENSOR_SCALE);
        _rightPot = new AnalogPotentiometer(Constants.AIO.CLIMB_RIGHT_SENSOR, Constants.Climb.SENSOR_SCALE);
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs)
    {
        inputs.leftExtension  = _leftPot.get() - _leftOffset;
        inputs.rightExtension = _rightPot.get() - _rightOffset;

        inputs.leftVolts  = _leftArmMotor.getMotorOutputVoltage();
        inputs.rightVolts = _rightArmMotor.getMotorOutputVoltage();
    }

    @Override
    public void setLeftVolts(double volts)
    {
        _leftArmMotor.set(VictorSPXControlMode.PercentOutput, volts / Constants.General.MOTOR_VOLTAGE);
    }

    @Override
    public void setRightVolts(double volts)
    {
        _rightArmMotor.set(VictorSPXControlMode.PercentOutput, volts / Constants.General.MOTOR_VOLTAGE);
    }

    @Override
    public void setLeftOffset(double offset)
    {
        _leftOffset = offset;
    }

    @Override
    public void setRightOffset(double offset)
    {
        _rightOffset = offset;
    }
}
