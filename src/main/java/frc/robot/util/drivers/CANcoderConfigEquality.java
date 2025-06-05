package frc.robot.util.drivers;

import com.ctre.phoenix6.configs.*;
import edu.wpi.first.math.MathUtil;

public class CANcoderConfigEquality {

    public static final boolean ENABLE_LOGGING_INEQ = true;

    public static final double TALON_CONFIG_EPSILON = 0.05;

    public static boolean isEqual(CANcoderConfiguration a, CANcoderConfiguration b)
    {
        return isEqual(a.MagnetSensor, b.MagnetSensor);
    }

    public static boolean isEqual(MagnetSensorConfigs a, MagnetSensorConfigs b)
    {
        return MathUtil.isNear(a.AbsoluteSensorDiscontinuityPoint,
            b.AbsoluteSensorDiscontinuityPoint, TALON_CONFIG_EPSILON)
            && MathUtil.isNear(a.MagnetOffset, b.MagnetOffset, TALON_CONFIG_EPSILON)
            && a.SensorDirection == b.SensorDirection;
    }

    private CANcoderConfigEquality()
    {}
}
