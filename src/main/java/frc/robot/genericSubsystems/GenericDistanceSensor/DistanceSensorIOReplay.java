package frc.robot.genericSubsystems.GenericDistanceSensor;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.drivers.CanDeviceId;

public class DistanceSensorIOReplay implements DistanceSensorIO {

    public DistanceSensorIOReplay()
    {}

    @Override
    public void updateInputs(DistanceSensorIOInputs inputs)
    {}

    @Override
    public void runCoast()
    {}

    @Override
    public void runBrake()
    {}

    @Override
    public void runVoltage(Voltage voltage)
    {}

    @Override
    public void runCurrent(Current current)
    {}

    @Override
    public void setG(double g)
    {}

    @Override
    public void runToPosition(Angle position)
    {}

    @Override
    public void runToVelocity(AngularVelocity velocity)
    {}

    @Override
    public void runMotionProfiledPosition(Angle position)
    {}

    @Override
    public void runMotionProfiledVelocity(AngularVelocity velocity)
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
            public DistanceSensorIntrinsics intrinsics()
            {
                return new DistanceSensorIntrinsics();
            }

            @Override
            public String name()
            {
                return "";
            }

            @Override
            public ControlType closedLoopControlType()
            {
                return ControlType.VOLTAGE;
            }
        };
    }

    @Override
    public void setPosition(Angle position)
    {}
}
