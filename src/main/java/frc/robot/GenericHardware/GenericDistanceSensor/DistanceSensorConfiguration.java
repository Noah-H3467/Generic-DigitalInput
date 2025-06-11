package frc.robot.GenericHardware.GenericDistanceSensor;

import com.ctre.phoenix6.configs.ProximityParamsConfigs;

import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import frc.robot.GenericHardware.GenericDistanceSensor.DistanceSensorIO.RoiFovConfigs;
import frc.robot.util.drivers.CanDeviceId;

public interface DistanceSensorConfiguration {
    public String name();

    public CanDeviceId id();

    // CANrange only
    public ProximityParamsConfigs proximityConfigs();

    public RoiFovConfigs roiFovConfigs();
    
    // LaserCAN only
    public RangingMode rangingMode();
    
    public TimingBudget timingBudget();
}
