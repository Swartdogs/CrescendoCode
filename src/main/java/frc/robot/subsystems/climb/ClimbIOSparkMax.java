// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.climb;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import frc.robot.Constants;

public class ClimbIOSparkMax implements ClimbIO
{
    public final CANSparkMax         _climbSparkMaxLeft;
    public final CANSparkMax         _climbSparkMaxRight;
    public final AnalogPotentiometer _potentiometerLeft;
    public final AnalogPotentiometer _potentiometerRight;

    public ClimbIOSparkMax()
    {
        _climbSparkMaxLeft  = new CANSparkMax(Constants.CAN.CLIMB_LEFT, MotorType.kBrushless);
        _climbSparkMaxRight = new CANSparkMax(Constants.CAN.CLIMB_RIGHT, MotorType.kBrushless);

        _potentiometerLeft  = new AnalogPotentiometer(Constants.AIO.CLIMB_LEFT_SENSOR);
        _potentiometerRight = new AnalogPotentiometer(Constants.AIO.CLIMB_RIGHT_SENSOR);
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs)
    {
        inputs.extensionLeft  = _potentiometerLeft.getValue(); // TODO, scaling factor work
        inputs.extensionRight = _potentiometerRight.getValue();

        inputs.appliedVoltsLeft  = _climbSparkMaxLeft.getAppliedOutput() * _climbSparkMaxLeft.getBusVoltage();
        inputs.appliedVoltsRight = _climbSparkMaxRight.getAppliedOutput() * _climbSparkMaxRight.getBusVoltage();
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
}
