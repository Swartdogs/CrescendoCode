package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.climb.Climb;

public final class ClimbCommands
{
    private ClimbCommands()
    {
    }

    public static Command setVoltage(Climb climb, DoubleSupplier leftSupplier, DoubleSupplier rightSupplier)
    {
        return climb.run(() ->
        {
            climb.setVoltageLeft(leftSupplier.getAsDouble() * Constants.General.MOTOR_VOLTAGE);
            climb.setVoltageRight(rightSupplier.getAsDouble() * Constants.General.MOTOR_VOLTAGE);
        });
    }

    public static Command setHeight(Climb climb, double setpoint)
    {
        return climb.runOnce(() -> climb.setHeight(setpoint));
    }
}
