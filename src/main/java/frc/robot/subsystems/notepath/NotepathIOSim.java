// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.notepath;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

/** Add your docs here. */
public class NotepathIOSim implements NotepathIO
{
    private DCMotorSim   _leaderNotepathSim   = new DCMotorSim(DCMotor.getNeo550(1), 6.75, 0.025);
    private DCMotorSim   _followerNotepathSim = new DCMotorSim(DCMotor.getNeo550(1), 6.75, 0.025);
    private DigitalInput _noteSensorSim       = new DigitalInput(Constants.DIO.NOTE_SENSOR);
    private double       _leaderVoltage;
    private double       _followerVoltage;

    @Override
    public void updateInputs(NotepathInputs inputs)
    {
        _leaderNotepathSim.update(Constants.General.LOOP_PERIOD_SECS);
        _followerNotepathSim.update(Constants.General.LOOP_PERIOD_SECS);

        inputs.leaderNotepathAppliedVolts = _leaderVoltage;
        inputs.leaderNotepathCurrentAmps  = new double[] { Math.abs(_leaderNotepathSim.getCurrentDrawAmps()) };

        inputs.followerNotepathAppliedVolts = _followerVoltage;
        inputs.followerNotepathCurrentAmps  = new double[] { Math.abs(_followerNotepathSim.getCurrentDrawAmps()) };
        inputs.sensorTripped                = !_noteSensorSim.get();
    }

    @Override
    public void setVoltage(double volts)
    {
        _leaderVoltage   = volts;
        _followerVoltage = volts;
        _leaderNotepathSim.setInputVoltage(_leaderVoltage);
        _followerNotepathSim.setInputVoltage(-_followerVoltage);
    }
}
