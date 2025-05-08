package frc.robot.genericSubsystems.GenericDigitalInput;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import lombok.Getter;

public class DigitalInput {
    @Getter
    private DigitalInputIO io;
    private DigitalInputIOInputsAutoLogged inputs = new DigitalInputIOInputsAutoLogged();

    private final Alert disconnectedAlert;

    public DigitalInput(DigitalInputIO io)
    {
        this.io = io;

        disconnectedAlert =
            new Alert("DigitalInput " + io.getConfiguration().name() + " disconnected", AlertType.kError);
    }

    public void update()
    {
        io.updateInputs(inputs);
        Logger.processInputs(io.getConfiguration().name(), inputs);

        disconnectedAlert.set(!isConnected());
    }

    /** Whether the DigitalInput is connected. */
    public boolean isConnected()
    {
        return inputs.connected;
    }

    /** Get the state of the DigitalInput. */
    public boolean get()
    {
        return inputs.state;
    }

    /** Get the analog trigger type */
    public int getAnalogTriggerTypeForRouting()
    {
        return inputs.analogTriggerType;
    }

    /** Get the channel of the DigitalInput. */
    public int getChannel()
    {
        return inputs.channel;
    }

}
