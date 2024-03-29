package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.trap.Trap;

public final class TrapCommands
{
    private TrapCommands()
    {
    }

    public static Command setVolts(Trap trap, DoubleSupplier ySupplier)
    {
        return trap.run(() -> trap.setVolts(ySupplier.getAsDouble() * Constants.ShooterBed.MAX_BED_VOLTS));
    }
}
