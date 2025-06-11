package frc.robot.GenericHardware.GenericDigitalInput;

import java.util.Optional;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.hal.SimDevice;

import frc.robot.Constants;

import lombok.Getter;

/**
 * The DigitalInputIOTalonFX class represents a DigitalInput interface for the TalonFX DigitalInput controller. It
 * integrates with beam break sensors and limit switches and provides methods for getting data from the DigitalInput,
 */
public class DigitalInputIOWPILib implements DigitalInputIO {

    public record DigitalInputIOConfiguration(
        String name,
        int portNumber)
        implements DigitalInputConfiguration {

        public DigitalInputIOConfiguration(
            Optional<String> name,
            int portNumber)
        {
            this(
                name.isEmpty() ? "id " + portNumber : name.get(),
                portNumber);
        }
    }

    @Getter
    private final String name;

    @Getter
    private final DigitalInputIOConfiguration configuration;

    private final DigitalInput m_DigitalInput;

    /**
     * Constructor for DigitalInputIOTalonFX that initializes the DigitalInput, its status signals, and applies
     * configurations for motion control.
     *
     * @param builder The TalonFXBuilder instance used to build the DigitalInputIOTalonFX object.
     */
    public DigitalInputIOWPILib(DigitalInputIOConfiguration config)
    {
        configuration = config;

        m_DigitalInput = new DigitalInput(config.portNumber);

        name = config.name();

        if (Constants.currentMode == Constants.Mode.SIM) {
            setSimDevice();
        }
    }

    /**
     * Updates the DigitalInput inputs for tracking the sensor's state and other WPILib supported values.
     *
     * @param inputs The DigitalInputIOInputs object where the updated values will be stored.
     */
    @Override
    public void updateInputs(DigitalInputIOInputs inputs)
    {
        // inputs.connected = ?

        // Updates the inputs with the DigitalInput values
        inputs.state = m_DigitalInput.get();
        inputs.analogTriggerType = m_DigitalInput.getAnalogTriggerTypeForRouting();
        inputs.channel = m_DigitalInput.getChannel();
    }

    /**
     * Get the state of the digital input
     */
    @Override
    public boolean get() {
        return m_DigitalInput.get();
    }

    /**
     * Close the digital input
     */
    @Override
    public void close()
    {
        m_DigitalInput.close();
    }

    /**
     * Indicates that the input is used by a simulated device.
     * TODO: Create the sim device earlier, but only when in simulation mode.
     */
    @Override
    public void setSimDevice()
    {
        m_DigitalInput.setSimDevice(SimDevice.create(name));
    }

}
