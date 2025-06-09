package frc.robot.GenericHardware.GenericDistanceSensor;

import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;

import au.grapplerobotics.interfaces.LaserCanInterface;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;

import frc.robot.GenericHardware.GenericDistanceSensor.DistanceSensorIOCANrange.DistanceSensorIOCANrangeConfiguration;
import frc.robot.GenericHardware.GenericDistanceSensor.DistanceSensorIOLaserCAN.DistanceSensorIOLaserCANConfiguration;
import frc.robot.util.drivers.CanDeviceId;

public class DistanceSensorIOReplay implements DistanceSensorIO {

    public DistanceSensorIOReplay()
    {}

    @Override
    public void updateInputs(DistanceSensorIOInputs inputs)
    {}



    @Override
    public DistanceSensorIOCANrangeConfiguration getCANrangeConfiguration()
    {
        return new DistanceSensorIOCANrangeConfiguration(
            "",
            new CanDeviceId(0),
            new CANrange(0, "replay"),
            new FovParamsConfigs(),
            new ProximityParamsConfigs(),
            new RegionOfInterest(0, 0, 0, 0),
            RangingMode.SHORT,
            TimingBudget.TIMING_BUDGET_20MS
        );
    }

    @Override
    public DistanceSensorIOLaserCANConfiguration getLaserCANConfiguration()
    {
        return new DistanceSensorIOLaserCANConfiguration(
            "",
            new CanDeviceId(0),
            new RegionOfInterest(0, 0, 0, 0),
            RangingMode.SHORT, 
            TimingBudget.TIMING_BUDGET_20MS
        );

    }
}
