package frc.robot.GenericHardware.GenericDistanceSensor;

import java.util.Optional;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.drivers.CanDeviceId;
import frc.robot.util.drivers.Phoenix6Util;
import lombok.Getter;

/**
 * The DistanceSensorIOCANrange class represents a DistanceSensor interface for the CANrange distance sensor. It
 * integrates with the CTRE Phoenix 6 hardware and provides methods for customizing the Distance Sensor,
 * including FOV and Proximity constants.
 */
public class DistanceSensorIOCANrange implements DistanceSensorIO {

    public record DistanceSensorIOConfiguration(
        String name, 
        CanDeviceId id,
        CANrange sensor,
        RoiFovConfigs roiFovConfigs,
        ProximityParamsConfigs proximityConfigs,
        RangingMode rangingMode,
        TimingBudget timingBudget) implements DistanceSensorConfiguration{

        public DistanceSensorIOConfiguration(
            Optional<String> name, 
            CanDeviceId id,
            RoiFovConfigs roiFovConfigs,
            ProximityParamsConfigs proximityConfigs,
            RangingMode rangingMode,
            TimingBudget timingBudget) 
        {
            this(
                name.orElse("id " + id.getDeviceNumber()), 
                id, 
                new CANrange(id.getDeviceNumber()),
                roiFovConfigs,
                proximityConfigs,
                rangingMode,
                timingBudget);
        }
    }
    private final String name;

    @Getter
    private final DistanceSensorIOConfiguration configuration;

    private final CANrange distanceSensor;

    private final StatusSignal<Distance> distance;
    private final StatusSignal<Distance> distanceStdDev;
    private final StatusSignal<Voltage> supplyVoltage;
    private final StatusSignal<Double> ambientSignal;
    private final StatusSignal<Angle> fovCenterX;
    private final StatusSignal<Angle> fovCenterY;
    private final StatusSignal<Angle> fovRangeX;
    private final StatusSignal<Angle> fovRangeY;
    private final StatusSignal<Boolean> isDetected;

    /**
     * Constructor for DistanceSensorIOCANrange that initializes the CANrange, its status signals, and applies
     * configurations for the ToF Field of view and proximity detection.
     *
     * @param builder The instance used to build the DistanceSensorIOCANrange object.
     */
    public DistanceSensorIOCANrange(DistanceSensorIOConfiguration config)
    {
        configuration = config;
        name = config.name();

        distanceSensor = config.sensor();
        CANrangeConfiguration configuration = new CANrangeConfiguration();
        configuration.FovParams.FOVCenterX = config.roiFovConfigs().centerX();
        configuration.FovParams.FOVCenterY = config.roiFovConfigs().centerY();
        configuration.FovParams.FOVRangeX = config.roiFovConfigs().width();
        configuration.FovParams.FOVRangeY = config.roiFovConfigs().length();

        configuration.ProximityParams = config.proximityConfigs();
        distanceSensor.getConfigurator().apply(configuration);

        distance = distanceSensor.getDistance();
        distanceStdDev = distanceSensor.getDistanceStdDev();
        supplyVoltage = distanceSensor.getSupplyVoltage();
        ambientSignal = distanceSensor.getAmbientSignal();
        fovCenterX = distanceSensor.getRealFOVCenterX();
        fovCenterY = distanceSensor.getRealFOVCenterY();
        fovRangeX = distanceSensor.getRealFOVRangeX();
        fovRangeY = distanceSensor.getRealFOVRangeY();
        isDetected = distanceSensor.getIsDetected();

        // Set update frequencies for the StatusSignals of interest
        Phoenix6Util.checkErrorAndRetry(
            () -> BaseStatusSignal.setUpdateFrequencyForAll(
                1000/config.timingBudget().asMilliseconds(),
                distance,
                distanceStdDev,
                supplyVoltage,
                isDetected,
                fovCenterX,
                fovCenterY,
                fovRangeX,
                fovRangeY));

        distanceSensor.optimizeBusUtilization(0, 1.0);

        Phoenix6Util.applyAndCheckConfiguration(distanceSensor, configuration);

    }

    /**
     * Updates the DistanceSensor inputs for tracking position, velocity, and other control parameters.
     *
     * @param inputs The DistanceSensorIOInputs object where the updated values will be stored.
     */
    @Override
    public void updateInputs(DistanceSensorIOInputs inputs)
    {
        // Refreshes the signals for position, velocity, and other values
        inputs.connected = BaseStatusSignal.refreshAll(
            distance,
            distanceStdDev,
            supplyVoltage,
            isDetected,
            fovCenterX,
            fovCenterY,
            fovRangeX,
            fovRangeY,
            ambientSignal)
            .isOK();

        // Updates the inputs with the DistanceSensor values
        inputs.distance = distance.getValue();
        inputs.distanceStdDev = distanceStdDev.getValue();
        inputs.appliedVoltage = supplyVoltage.getValue();
        inputs.isDetected = isDetected.getValue();
        inputs.fovCenterX = fovCenterX.getValue();
        inputs.fovCenterY = fovCenterY.getValue();
        inputs.fovRangeX = fovRangeX.getValue();
        inputs.fovRangeY = fovRangeY.getValue();
        inputs.ambientSignal = ambientSignal.getValue();
    }


    /**
     * Updates the current settings that determine whether the CANrange 'detects' an object.
     * Call this method to achieve the desired performance.
     *
     * @param newProximityThreshold The threshold for object detection. Between 0.0 and 4.0
     * @param newProximityHysteresis The hysteresis value for object detection. Between 0.0 and 4.0
     * @param newMinSignalStrengthForValidMeasurement The minimum signal strength for a valid measurement. 
     * If the signal strength is particularly low, this typically means
     * the object is far away and there's fewer total samples to derive
     * the distance from. Set this value to be below the lowest strength
     * you see when you're detecting an object with the CANrange; the
     * default of 2500 is typically acceptable in most cases.
     */
    private void setProximityParams(double newProximityThreshold, double newProximityHysteresis, double newMinSignalStrengthForValidMeasurement)
    {
        CANrangeConfiguration config = new CANrangeConfiguration();
        config.ProximityParams.ProximityThreshold = newProximityThreshold;
        config.ProximityParams.ProximityHysteresis = newProximityHysteresis;
        config.ProximityParams.MinSignalStrengthForValidMeasurement = newMinSignalStrengthForValidMeasurement;
        distanceSensor.getConfigurator().apply(config);
    }

    /**
     * Updates the current settings that determine whether the center and extent of the CANrange's view
     * Call this method to achieve the desired performance.
     *
     * @param newFOVCenterX Specifies the target center of the Field of View in the X direction, between +/- 11.8
     * @param newFOVCenterY Specifies the target center of the Field of View in the Y direction, between +/- 11.8
     * @param newFOVRangeX Specifies the target range of the Field of View in the X direction. The magnitude of this is capped to abs(27 - 2*FOVCenterY).
     * @param newFOVRangeY Specifies the target range of the Field of View in the Y direction. The magnitude of this is capped to abs(27 - 2*FOVCenterY).
     * 
     */
    private void setFOVParams(double newFOVCenterX, double newFOVCenterY, double newFOVRangeX, double newFOVRangeY)
    {
        CANrangeConfiguration config = new CANrangeConfiguration();
        config.FovParams.FOVCenterX = newFOVCenterX;
        config.FovParams.FOVCenterY = newFOVCenterY;
        config.FovParams.FOVRangeX = newFOVRangeX;
        config.FovParams.FOVRangeY = newFOVRangeY;
        distanceSensor.getConfigurator().apply(config);
    }

}