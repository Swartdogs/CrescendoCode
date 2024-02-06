// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.climb;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import frc.robot.Constants;

public class ClimbIOVictorSPX implements ClimbIO
{
    public final VictorSPX           _climbVictorSPXLeft;
    public final VictorSPX           _climbVictorSPXRight;

    public final AnalogPotentiometer _potentiometerLeft;
    public final AnalogPotentiometer _potentiometerRight;

    public final double _leftOffset = Constants.Climb.LEFT_ZERO_OFFSET;
    public final double _rightOffset = Constants.Climb.RIGHT_ZERO_OFFSET;

    public ClimbIOVictorSPX()
    {
        _climbVictorSPXLeft  = new VictorSPX(Constants.CAN.CLIMB_LEFT);
        _climbVictorSPXRight = new VictorSPX(Constants.CAN.CLIMB_RIGHT);

        _potentiometerLeft  = new AnalogPotentiometer(Constants.AIO.CLIMB_LEFT_SENSOR);
        _potentiometerRight = new AnalogPotentiometer(Constants.AIO.CLIMB_RIGHT_SENSOR);
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs)
    {
        inputs.extensionLeft  = _potentiometerLeft.get(); // TODO, scaling factor work
        inputs.extensionRight = _potentiometerRight.get();

        inputs.appliedVoltsLeft  = _climbVictorSPXLeft.getMotorOutputVoltage();
        inputs.appliedVoltsRight = _climbVictorSPXRight.getMotorOutputVoltage();
    }

    @Override
    public void setVoltageLeft(double volts)
    {
        _climbVictorSPXLeft.set(VictorSPXControlMode.PercentOutput, volts / Constants.General.MOTOR_VOLTAGE);
    }

    @Override
    public void setVoltageRight(double volts)
    {
        _climbVictorSPXRight.set(VictorSPXControlMode.PercentOutput, volts / Constants.General.MOTOR_VOLTAGE);
    }
}
