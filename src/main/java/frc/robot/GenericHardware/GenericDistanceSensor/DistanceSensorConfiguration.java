package frc.robot.GenericHardware.GenericDistanceSensor;

import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;

import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;

import frc.robot.util.drivers.CanDeviceId;

public interface DistanceSensorConfiguration {
    public String name();

    public CanDeviceId id();

    public ProximityParamsConfigs proximityConfigs();

    public FovParamsConfigs fovConfigs();

    public RegionOfInterest roiConfigs();
    
    public RangingMode rangingMode();
    
    public TimingBudget timingBudget();
}
