package frc.robot.subsystems.trap;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.robot.Constants;

public class TrapSnowblower implements TrapIO
{
    private final VictorSPX _trapMotor;

    public TrapSnowblower()
    {
        _trapMotor = new VictorSPX(Constants.CAN.TRAP);
    }

    @Override
    public void updateInputs(TrapIOInputs inputs)
    {
        inputs.trapVolts = _trapMotor.getMotorOutputVoltage();
    }

    @Override
    public void setVolts(double volts)
    {
        _trapMotor.set(VictorSPXControlMode.PercentOutput, volts / Constants.General.MOTOR_VOLTAGE);
    }
}
