// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.AnalogInputSim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SolenoidSim;
import frc.robot.Constants;

public class ClimbIOSim implements ClimbIO {
  private DCMotorSim _leftSim = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.003); // Find values
  private DCMotorSim _rightSim = new DCMotorSim(DCMotor.getNEO(1), 0, 0); // Find values

  private AnalogInputSim _leftExtensionSim = new AnalogInputSim(0);
  private AnalogInputSim _rightExtensionSim = new AnalogInputSim(1);

  private SolenoidSim _leftSolenoidSim = new SolenoidSim(Constants.Pnuematics.MODULE_TYPE, 0);
  private SolenoidSim _rightSolenoidSim = new SolenoidSim(Constants.Pnuematics.MODULE_TYPE, 1);

  private double _leftAppliedVolts = 0.0;
  private double _rightAppliedVolts = 0.0;

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    _leftSim.update(Constants.General.LOOP_PERIOD_SECS);
    _rightSim.update(Constants.General.LOOP_PERIOD_SECS);

    inputs.extensionLeft = _leftExtensionSim.getVoltage(); // CHANGE!!!
    inputs.extensionRight = _rightExtensionSim.getVoltage();

    inputs.lockStateLeft = _leftSolenoidSim.getOutput();
    inputs.lockStateRight = _rightSolenoidSim.getOutput();

    inputs.appliedVoltsLeft = _leftAppliedVolts;
    inputs.appliedVoltsRight = _rightAppliedVolts;

    inputs.currentAmpsLeft = new double[] {Math.abs(_leftSim.getCurrentDrawAmps())};
    inputs.currnetAmpsRight = new double[] {Math.abs(_rightSim.getCurrentDrawAmps())};
  }

  public void setLeftVoltage(double volts) {
    _leftAppliedVolts =
        MathUtil.clamp(volts, -12.0, 12.0); // Figure out how this transfers to simulation
    _leftSim.setInputVoltage(_leftAppliedVolts);
  }

  public void setRightVoltage(double volts) {
    _rightAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    _rightSim.setInputVoltage(_rightAppliedVolts);
  }
}
