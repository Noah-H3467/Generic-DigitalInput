package frc.robot.genericSubsystems.GenericDistanceSensor;

import frc.robot.genericSubsystems.GenericDistanceSensor.DistanceSensorIO.ControlType;
import frc.robot.genericSubsystems.GenericDistanceSensor.DistanceSensorIO.DistanceSensorIntrinsics;
import frc.robot.util.drivers.CanDeviceId;

public interface DistanceSensorConfiguration {
    public String name();

    public CanDeviceId id();

    public DistanceSensorIntrinsics intrinsics();

    public ControlType closedLoopControlType();
}
