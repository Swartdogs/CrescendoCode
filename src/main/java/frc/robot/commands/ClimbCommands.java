package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.pneumaticclimb.PneumaticClimb;
import frc.robot.subsystems.pneumaticclimb.PneumaticClimbIO;

public final class ClimbCommands
{
    private ClimbCommands()
    {
    }

    public static Command setVolts(Climb climb, DoubleSupplier leftVolts, DoubleSupplier rightVolts)
    {
        return climb.run(() ->
        {
            climb.setLeftVolts(leftVolts.getAsDouble() * Constants.General.MOTOR_VOLTAGE);
            climb.setRightVolts(rightVolts.getAsDouble() * Constants.General.MOTOR_VOLTAGE);
        }).finallyDo(() -> climb.stop());
    }

    public static Command setHeight(Climb climb, double height)
    {
        return climb.runOnce(() -> climb.setHeight(height));
    }

    public static Command set(PneumaticClimb climb, boolean state)
    {
        return climb.runOnce(() -> climb.set(state));
    }
}
