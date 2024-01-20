package frc.robot.subsystems.Intake;

import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Intake {
  private IntakeIO _io;
  private boolean _isIntaking = false;
  private final IntakeIOInputsAutoLogged _inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    _io = io;
  }

  public void periodic() {
    _io.updateInputs(_inputs);
    Logger.processInputs("Intake/", _inputs);
    Logger.recordOutput("Intake/", _isIntaking);

  }

  public void startIntake() {
    _io.setVoltage(Constants.Intake.INTAKE_SPEED);
    _isIntaking = true;
  }

  public void stopIntake() {
    _io.setVoltage(0);
    _isIntaking = false;
  }

  public double getIntakeSpeed() {
    return _inputs.intakeSpeed;
  }
}
