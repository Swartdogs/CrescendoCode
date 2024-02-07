package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climb.Climb;

public final class ClimbCommands
{
    private ClimbCommands()
    {
    }

    public static Command setLeftVoltage(Climb climb, DoubleSupplier xSupplier)
    {
        return Commands.runOnce(() -> climb.setVoltageLeft(xSupplier.getAsDouble() * 12));
    }

    public static Command setRightVoltage(Climb climb, DoubleSupplier ySupplier)
    {
        return Commands.runOnce(() -> climb.setVoltageRight(ySupplier.getAsDouble() * 12));
    }

    public static Command setHeight(Climb climb, double setpoint)
    {
        return Commands.runOnce(() ->  climb.setHeight(setpoint)); //TODO: Need isFinished?
    }
}
