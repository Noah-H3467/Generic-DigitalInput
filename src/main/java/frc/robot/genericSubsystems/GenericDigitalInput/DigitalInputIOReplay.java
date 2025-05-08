package frc.robot.genericSubsystems.GenericDigitalInput;

public class DigitalInputIOReplay implements DigitalInputIO {

    public DigitalInputIOReplay()
    {}

    @Override
    public void updateInputs(DigitalInputIOInputs inputs)
    {}

    @Override
    public boolean get()
    {
        return false;
    }

    @Override
    public void setSimDevice()
    {}

    @Override
    public DigitalInputConfiguration getConfiguration()
    {
        return new DigitalInputConfiguration() {

            @Override
            public String name()
            {
                return "";
            }

        };
    }

    @Override
    public void close()
    {}
}
