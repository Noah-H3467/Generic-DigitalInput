package frc.robot.GenericHardware.GenericDistanceSensor;

import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;

import frc.robot.util.drivers.CanDeviceId;

public interface DistanceSensorConfiguration {
    public String name();

    public CanDeviceId id();

    public ProximityParamsConfigs proximityConfigs();

    public FovParamsConfigs fovConfigs();
}
