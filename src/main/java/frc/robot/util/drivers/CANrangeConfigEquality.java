package frc.robot.util.drivers;

import com.ctre.phoenix6.configs.*;
import edu.wpi.first.math.MathUtil;

public class CANrangeConfigEquality {

    public static final boolean ENABLE_LOGGING_INEQ = true;

    public static final double CANRANGE_CONFIG_EPSILON = 0.05;

    public static boolean isEqual(CANrangeConfiguration a, CANrangeConfiguration b)
    {
        return isEqual(a.CustomParams, b.CustomParams)
            && isEqual(a.FovParams, b.FovParams)
            && isEqual(a.ProximityParams, b.ProximityParams)
            && (a.FutureProofConfigs == b.FutureProofConfigs);
    }

    public static boolean isEqual(FovParamsConfigs a, FovParamsConfigs b)
    {
        boolean val = MathUtil.isNear(a.FOVCenterX, b.FOVCenterX, CANRANGE_CONFIG_EPSILON)
            && MathUtil.isNear(a.FOVCenterY, b.FOVCenterY, CANRANGE_CONFIG_EPSILON)
            && MathUtil.isNear(a.FOVRangeX, b.FOVRangeX, CANRANGE_CONFIG_EPSILON)
            && MathUtil.isNear(a.FOVRangeY, b.FOVRangeY, CANRANGE_CONFIG_EPSILON);
        
        if (ENABLE_LOGGING_INEQ && !val) {
            System.out.println("FovParamsConfigs not equal");
        }
        return val;
    }

    public static boolean isEqual(ProximityParamsConfigs a, ProximityParamsConfigs b)
    {
        boolean val = MathUtil.isNear(a.MinSignalStrengthForValidMeasurement, b.MinSignalStrengthForValidMeasurement, CANRANGE_CONFIG_EPSILON)
            && MathUtil.isNear(a.ProximityHysteresis, b.ProximityHysteresis, CANRANGE_CONFIG_EPSILON)
            && MathUtil.isNear(a.ProximityThreshold, b.ProximityThreshold, CANRANGE_CONFIG_EPSILON);
        
        if (ENABLE_LOGGING_INEQ && !val) {
            System.out.println("ProximityParamsConfigs not equal");
        }
        return val;
    }

    public static boolean isEqual(CustomParamsConfigs a, CustomParamsConfigs b)
    {
        boolean val = MathUtil.isNear(a.CustomParam0, b.CustomParam0, CANRANGE_CONFIG_EPSILON)
            && MathUtil.isNear(a.CustomParam1, b.CustomParam1, CANRANGE_CONFIG_EPSILON);
        
        if (ENABLE_LOGGING_INEQ && !val) {
            System.out.println("CustomParamsConfigs not equal");
        }
        return val;
    }

    private CANrangeConfigEquality()
    {}
}
