package frc.robot.genericSubsystems.GenericDistanceSensor;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.NewtonMeters;
import static edu.wpi.first.units.Units.Ohms;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.TorqueUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Resistance;
import edu.wpi.first.units.measure.Temperature;
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
        /** The actual center of the FOV in the X direction */
        public Angle fovCenterX = Radians.of(0.0);
        /** The actual center of the FOV in the Y direction */
        public Angle fovCenterY = Radians.of(0.0);
        /** The actual range of the FOV in the X direction */
        public Angle fovRangeX = Radians.of(0.0);
        /** The actual range of the FOV in the Y direction */
        public Angle fovRangeY = Radians.of(0.0);
        /** The applied voltage the DistanceSensor */
        public Voltage appliedVoltage = Volts.of(0.0);

    }

    public void updateInputs(DistanceSensorIOInputs inputs);

    public DistanceSensorConfiguration getConfiguration();

    public void setPosition(Angle position);

    // ------------------ Open Loop Control ------------------

    /**
     * Allows the DistanceSensor to move freely
     */
    public void runCoast();

    /**
     * Passively prevents the DistanceSensor shaft from rotating
     */
    public void runBrake();

    /**
     * Runs the DistanceSensor with a specified voltage (open-loop control).
     *
     * @param voltage The voltage to apply to the DistanceSensor in volts.
     */
    public void runVoltage(Voltage voltage);

    /**
     * Runs the DistanceSensor with a specified current (open-loop torque control).
     *
     * @param current The current to apply to the DistanceSensor in amperes.
     */
    public void runCurrent(Current current);

    // ------------------ Feedforward Terms ------------------

    /**
     * Sets the gravity feedforward gain (G) in terms of the current output mode (voltage or
     * current).
     *
     * @param g Gravity feedforward gain.
     */
    public void setG(double g);

    // ------------------ Position and Velocity Control ------------------

    /**
     * Runs the DistanceSensor to a target position in rotations using closed-loop control.
     *
     * @param positionRotations Target position in rotations.
     */
    public void runToPosition(Angle position);
    
}
