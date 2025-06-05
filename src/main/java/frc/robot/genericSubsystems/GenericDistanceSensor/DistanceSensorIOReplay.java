package frc.robot.genericSubsystems.GenericDistanceSensor;

import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
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
            public CanDeviceId id()
            {
                return new CanDeviceId(0);
            }


            @Override
            public String name()
            {
                return "";
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
        };
    }


}
