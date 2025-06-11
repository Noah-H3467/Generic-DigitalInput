package frc.robot.GenericHardware.GenericDistanceSensor;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import lombok.Getter;

public class DistanceSensor {
    @Getter
    private DistanceSensorIO io;
    private DistanceSensorIOInputsAutoLogged inputs = new DistanceSensorIOInputsAutoLogged();

    private DoubleSupplier gravitySupplier = () -> 0;

    private final Alert disconnectedAlert;

    public DistanceSensor(DistanceSensorIO io)
    {
        this.io = io;

        disconnectedAlert =
            new Alert("DistanceSensor " + io.getConfiguration().name() + " disconnected", AlertType.kError);
    }

    public void update()
    {
        io.updateInputs(inputs);
        Logger.processInputs(io.getConfiguration().name(), inputs);

        disconnectedAlert.set(!isConnected());
    }

    /** Whether the DistanceSensor is connected. */
    public boolean isConnected()
    {
        return inputs.connected;
    }

    /** Object within configured proximity. */
    public boolean isDetected()
    {
        return inputs.isDetected;
    }

    /** Distance from DistanceSensor to the nearest object in the configured view. */
    public Distance getDistance()
    {
        return inputs.distance;
    }

    /** Standard Deviation of the CANrange measurement. */
    public Distance getDistanceStdDev()
    {
        return inputs.distanceStdDev;
    }

    /** The amount of ambient infared light detected by the DistanceSensor. */
    public double getAmbientSignal()
    {
        return inputs.ambientSignal;
    }

    /** CANrange: Actual center of the FOV in the X direction. */
    public Angle getRealFOVCenterX()
    {
        return inputs.fovCenterX;
    }

    /** Actual center of the FOV in the Y direction. */
    public Angle getRealFOVCenterY()
    {
        return inputs.fovCenterY;
    }

    /** Actual range of the FOV in the X direction. */
    public Angle getRealFOVRangeX()
    {
        return inputs.fovRangeX;
    }

    /** Actual range of the FOV in the Y direction. */
    public Angle getRealFOVRangeY()
    {
        return inputs.fovRangeY;
    }

    /** LaserCAN: Actual center of the ROI in the X direction. */
    public int getRegionOfInterestX()
    {
        return inputs.roiX;
    }

    /** Actual center of the ROI in the Y direction. */
    public int getRegionOfInterestY()
    {
        return inputs.roiY;
    }

    /** Actual width of the ROI. */
    public int getRegionOfInterestWidth()
    {
        return inputs.roiW;
    }

    /** Actual height of the ROI. */
    public int getRegionOfInterestHeight()
    {
        return inputs.roiH;
    }

}
