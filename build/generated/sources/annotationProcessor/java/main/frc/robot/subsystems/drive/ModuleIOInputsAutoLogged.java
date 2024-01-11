package frc.robot.subsystems.drive;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ModuleIOInputsAutoLogged extends ModuleIO.ModuleIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("DrivePositionRad", drivePositionRad);
    table.put("DriveVelocityRadPerSec", driveVelocityRadPerSec);
    table.put("DriveAppliedVolts", driveAppliedVolts);
    table.put("DriveCurrentAmps", driveCurrentAmps);
    table.put("TurnAbsolutePosition", turnAbsolutePosition);
    table.put("TurnPosition", turnPosition);
    table.put("TurnVelocityRadPerSec", turnVelocityRadPerSec);
    table.put("TurnAppliedVolts", turnAppliedVolts);
    table.put("TurnCurrentAmps", turnCurrentAmps);
  }

  @Override
  public void fromLog(LogTable table) {
    drivePositionRad = table.get("DrivePositionRad", drivePositionRad);
    driveVelocityRadPerSec = table.get("DriveVelocityRadPerSec", driveVelocityRadPerSec);
    driveAppliedVolts = table.get("DriveAppliedVolts", driveAppliedVolts);
    driveCurrentAmps = table.get("DriveCurrentAmps", driveCurrentAmps);
    turnAbsolutePosition = table.get("TurnAbsolutePosition", turnAbsolutePosition);
    turnPosition = table.get("TurnPosition", turnPosition);
    turnVelocityRadPerSec = table.get("TurnVelocityRadPerSec", turnVelocityRadPerSec);
    turnAppliedVolts = table.get("TurnAppliedVolts", turnAppliedVolts);
    turnCurrentAmps = table.get("TurnCurrentAmps", turnCurrentAmps);
  }

  public ModuleIOInputsAutoLogged clone() {
    ModuleIOInputsAutoLogged copy = new ModuleIOInputsAutoLogged();
    copy.drivePositionRad = this.drivePositionRad;
    copy.driveVelocityRadPerSec = this.driveVelocityRadPerSec;
    copy.driveAppliedVolts = this.driveAppliedVolts;
    copy.driveCurrentAmps = this.driveCurrentAmps.clone();
    copy.turnAbsolutePosition = this.turnAbsolutePosition;
    copy.turnPosition = this.turnPosition;
    copy.turnVelocityRadPerSec = this.turnVelocityRadPerSec;
    copy.turnAppliedVolts = this.turnAppliedVolts;
    copy.turnCurrentAmps = this.turnCurrentAmps.clone();
    return copy;
  }
}
