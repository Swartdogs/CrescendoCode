package frc.robot.subsystems.flywheel;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class FlywheelIOInputsAutoLogged extends FlywheelIO.FlywheelIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("PositionRad", positionRad);
    table.put("VelocityRadPerSec", velocityRadPerSec);
    table.put("AppliedVolts", appliedVolts);
    table.put("CurrentAmps", currentAmps);
  }

  @Override
  public void fromLog(LogTable table) {
    positionRad = table.get("PositionRad", positionRad);
    velocityRadPerSec = table.get("VelocityRadPerSec", velocityRadPerSec);
    appliedVolts = table.get("AppliedVolts", appliedVolts);
    currentAmps = table.get("CurrentAmps", currentAmps);
  }

  public FlywheelIOInputsAutoLogged clone() {
    FlywheelIOInputsAutoLogged copy = new FlywheelIOInputsAutoLogged();
    copy.positionRad = this.positionRad;
    copy.velocityRadPerSec = this.velocityRadPerSec;
    copy.appliedVolts = this.appliedVolts;
    copy.currentAmps = this.currentAmps.clone();
    return copy;
  }
}
