package frc.robot.genericSubsystems.GenericDigitalInput;

import org.littletonrobotics.junction.AutoLog;

public interface DigitalInputIO {

    @AutoLog
    abstract class DigitalInputIOInputs {
        /** Whether the DigitalInput is connected. */
        public boolean connected = false;
        /** DigitalInput getter. */
        public boolean state = false;
        /** DigitalInput velocity. */
        public int analogTriggerType = 0;
        /** Voltage applied to the DigitalInput. */
        public int channel = 0;
    }

    public void updateInputs(DigitalInputIOInputs inputs);

    public DigitalInputConfiguration getConfiguration();

    /**
     * Gets the state of the DigitalInput.
     */
    public boolean get();

    /**
     * Indicates this input is used by a simulated device.
     */
    public void setSimDevice();

    /**
     * Closes the Digital Input and resources.
     */
    public void close();
}
