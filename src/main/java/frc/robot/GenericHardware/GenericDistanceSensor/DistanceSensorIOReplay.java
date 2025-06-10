package frc.robot.GenericHardware.GenericDistanceSensor;

import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;

import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import frc.robot.util.drivers.CanDeviceId;

public class DistanceSensorIOReplay implements DistanceSensorIO {

    public DistanceSensorIOReplay()
    {}

    @Override
    public void updateInputs(DistanceSensorIOInputs inputs)
    {}



    @Override
    public DistanceSensorConfiguration getConfiguration()
    {
        return new DistanceSensorConfiguration() {

            @Override
            public String name()
            {
                return "";
            }

            @Override
            public CanDeviceId id()
            {
                return new CanDeviceId(0, "");
            }

            @Override
            public FovParamsConfigs fovConfigs()
            {
                return new FovParamsConfigs();
            }

            @Override
            public ProximityParamsConfigs proximityConfigs()
            {
                return new ProximityParamsConfigs();
            }

            @Override
            public RegionOfInterest roiConfigs()
            {
                return new RegionOfInterest(0, 0, 0, 0);
            }
            @Override

            public RangingMode rangingMode()
            {
                return RangingMode.SHORT;
            }
            @Override
            public TimingBudget timingBudget()
            {
                return TimingBudget.TIMING_BUDGET_20MS;
            }

        };
    }

}
