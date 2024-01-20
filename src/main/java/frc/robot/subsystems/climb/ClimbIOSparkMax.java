// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class ClimbIOSparkMax implements ClimbIO
{
    public final CANSparkMax _climbSparkMax;
    public final AnalogInput _potentiometer;
    public final Solenoid    _solenoid;

    public ClimbIOSparkMax(int climbCanId, int potentiometerChannel, int solenoidId)
    {
        _climbSparkMax = new CANSparkMax(climbCanId, MotorType.kBrushless);
        _potentiometer = new AnalogInput(potentiometerChannel);
        _solenoid      = new Solenoid(PneumaticsModuleType.CTREPCM, solenoidId);
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs)
    {
        inputs.measuredExtension = _potentiometer.getValue(); // TODO, scaling factor work
        inputs.lockState         = _solenoid.get();
    }

    @Override
    public void setVoltage(double volts)
    {
        _climbSparkMax.setVoltage(volts);
    }

    @Override
    public void setLockState(boolean enable)
    {
        _solenoid.set(enable);
    }
}
