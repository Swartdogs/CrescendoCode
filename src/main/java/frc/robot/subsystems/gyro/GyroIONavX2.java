package frc.robot.subsystems.gyro;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI.Port;

public class GyroIONavX2 implements GyroIO
{
    private final AHRS _gyro = new AHRS(Port.kMXP);

    public GyroIONavX2()
    {
        _gyro.reset();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs)
    {
        inputs.yawPosition          = Rotation2d.fromDegrees(-_gyro.getYaw());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(-_gyro.getRate());
        inputs.rollPosition         = Rotation2d.fromDegrees(_gyro.getRoll());
    }
}
