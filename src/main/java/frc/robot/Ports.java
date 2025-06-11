package frc.robot;

import frc.robot.util.drivers.CanDeviceId;

public class Ports {
    public static final int BEAM_BREAK_PORT = 0;
    public static final int BEAM_BREAK_SUB_PORT = 1;

    public static final CanDeviceId EXAMPLE_STAGE_PORT = new CanDeviceId(2, "rio");
    public static final CanDeviceId EXAMPLE_STAGE_CANRANGE = new CanDeviceId(3, "rio");
}
