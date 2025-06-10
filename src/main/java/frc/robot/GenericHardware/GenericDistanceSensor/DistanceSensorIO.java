package frc.robot.GenericHardware.GenericDistanceSensor;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;
import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

public interface DistanceSensorIO {

    @AutoLog
    abstract class DistanceSensorIOInputs {
        /** Whether the DistanceSensor is connected. */
        public boolean connected = false;
        /** Whether the DistanceSensor is detected. */
        public boolean isDetected = false;
        /** Distance from the Distance sensor to the nearest object */
        public Distance distance = null;
        /** Standard deviation of the distance sensor measurement*/
        public Distance distanceStdDev = null;
        /** The amount of ambient infrared light detected by the DistanceSensor. */
        public double ambientSignal = 0.0;
        /** CANRange - The actual center of the FOV in the X direction */
        public Angle fovCenterX = Radians.of(0.0);
        /** The actual center of the FOV in the Y direction */
        public Angle fovCenterY = Radians.of(0.0);
        /** The actual range of the FOV in the X direction */
        public Angle fovRangeX = Radians.of(0.0);
        /** The actual range of the FOV in the Y direction */
        public Angle fovRangeY = Radians.of(0.0);
        /** LaserCAN -The center of the Region of Interest */
        public int roiX = 0;
        public int roiY = 0;
        /** LaserCAN - the width and height of the region of interest */
        public int roiW = 0;
        public int roiH = 0;
        /** CANrange? The applied voltage the DistanceSensor */
        public Voltage appliedVoltage = Volts.of(0.0);
        /** Whether the LaserCAN is at short or long mode. Short mode is up to 1.3 m and less susceptible to ambient light*/
        public boolean isShortMode = true;

    }

    public void updateInputs(DistanceSensorIOInputs inputs);

    public DistanceSensorConfiguration getConfiguration();
    
}
