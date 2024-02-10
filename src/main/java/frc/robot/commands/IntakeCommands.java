package frc.robot.commands;

import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public final class IntakeCommands
{
    private IntakeCommands()
    {
    }

    public static Command startIntake(Intake intake)
    {
        return intake.runOnce(() -> intake.set(Value.kOn));
    }

    public static Command stopIntake(Intake intake)
    {
        return intake.runOnce(() -> intake.set(Value.kOff));
    }

    public static Command reverseIntake(Intake intake)
    {
        return intake.runOnce(() -> intake.set(Value.kReverse));
    }
}
