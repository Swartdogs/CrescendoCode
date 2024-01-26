// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class ShooterBedIOSim implements ShooterBedIO
{
    private DCMotorSim _leftBedSim = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.025);
    private DCMotorSim _rightBedSim = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.025);

    private double _bedAppliedVolts = 0.0;

    @Override
    public void updateInputs(ShooterBedIOInputs inputs)
    {
        _leftBedSim.update(Constants.General.LOOP_PERIOD_SECS);
        _rightBedSim.update(Constants.General.LOOP_PERIOD_SECS);

        inputs.bedAngle = new Rotation2d(_leftBedSim.getAngularPositionRad());
        inputs.bedAppliedVolts = _bedAppliedVolts;
        inputs.bedCurrentAmps = new double[]
        { Math.abs(_leftBedSim.getCurrentDrawAmps()) };
    }

    @Override
    public void setVoltage(double volts)
    {
        _leftBedSim.setInputVoltage(volts);
        _rightBedSim.setInputVoltage(volts);
        _bedAppliedVolts = volts;
    }
}
