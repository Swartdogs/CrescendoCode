// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;

public class ClimbIOSparkMax implements ClimbIO
{
    public final CANSparkMax _climbSparkMaxLeft;
    public final CANSparkMax _climbSparkMaxRight;

    public final AnalogInput _potentiometerLeft;
    public final AnalogInput _potentiometerRight;
    
    public final Solenoid _solenoidLeft;
    public final Solenoid _solenoidRight;

    public ClimbIOSparkMax
    (
        int climbCanIdLeft, 
        int climbCanIdRight, 
        
        int potentiometerChannelLeft, 
        int potentiometerChannelRight, 
        
        int solenoidIdLeft, 
        int solenoidIdRight
    )
    {
        _climbSparkMaxLeft  = new CANSparkMax(climbCanIdLeft, MotorType.kBrushless);
        _climbSparkMaxRight = new CANSparkMax(climbCanIdRight, MotorType.kBrushless);

        _potentiometerLeft  = new AnalogInput(potentiometerChannelLeft);
        _potentiometerRight = new AnalogInput(potentiometerChannelRight);
        
        _solenoidLeft  = new Solenoid(Constants.Pnuematics.MODULE_TYPE, Constants.Pnuematics.SOLENOID_ID_LEFT);
        _solenoidRight = new Solenoid(Constants.Pnuematics.MODULE_TYPE, Constants.Pnuematics.SOLENOID_ID_RIGHT);
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs)
    {
        inputs.extensionLeft  = _potentiometerLeft.getValue(); // TODO, scaling factor work
        inputs.extensionRight = _potentiometerRight.getValue();

        inputs.lockStateLeft  = !_solenoidLeft.get();
        inputs.lockStateRight = !_solenoidRight.get();

        inputs.appliedVoltsLeft  = _climbSparkMaxLeft.getAppliedOutput() * _climbSparkMaxLeft.getBusVoltage();
        inputs.appliedVoltsRight = _climbSparkMaxRight.getAppliedOutput() * _climbSparkMaxRight.getBusVoltage();

        inputs.currentAmpsLeft  = new double[]{_climbSparkMaxLeft.getOutputCurrent()};
        inputs.currnetAmpsRight = new double[]{_climbSparkMaxRight.getOutputCurrent()};
    }

    @Override
    public void setVoltageLeft(double volts)
    {
        _climbSparkMaxLeft.setVoltage(volts);
    }

    @Override
    public void setVoltageRight(double volts)
    {
        _climbSparkMaxRight.setVoltage(volts);
    }

    @Override
    public void setLockStateLeft(boolean enable, ClimbIOInputs inputs)
    {
        if(inputs.lockStateLeft != enable)
        {
            _solenoidLeft.set(!enable);
        }
    }

    @Override
    public void setLockStateRight(boolean enable, ClimbIOInputs inputs)
    {
        if(inputs.lockStateRight != enable)
        {
            _solenoidRight.set(!enable);
        }
    }
}
