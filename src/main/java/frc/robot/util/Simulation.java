package frc.robot.util;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public final class Simulation
{
    private Simulation()
    {

    }

    public static DCMotor windowMotor()
    {
        return new DCMotor(12, 9.2, 16.3, 1.6, Units.rotationsPerMinuteToRadiansPerSecond(90), 1);
    }
}
