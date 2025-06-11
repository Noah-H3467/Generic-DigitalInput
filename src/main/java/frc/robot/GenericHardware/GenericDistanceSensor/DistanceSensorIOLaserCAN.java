package frc.robot.GenericHardware.GenericDistanceSensor;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;

import java.util.Optional;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import static edu.wpi.first.units.Units.*;

import frc.robot.Constants;
import frc.robot.util.drivers.CanDeviceId;
import lombok.Getter;

/**
 * The DistanceSensorIOLaserCAN class represents a DistanceSensor interface for the LaserCAN DistanceSensor controller. It
 * integrates with Thrifty's hardware and provides methods for controlling the DistanceSensor.
 */
public class DistanceSensorIOLaserCAN implements DistanceSensorIO {
    public record DistanceSensorIOConfiguration(
        String name, 
        CanDeviceId id,
        ProximityParamsConfigs proximityConfigs,
        RoiFovConfigs roiFovConfigs,
        RangingMode rangingMode,
        TimingBudget timingBudget) implements DistanceSensorConfiguration{

        public DistanceSensorIOConfiguration(
            Optional<String> name, 
            CanDeviceId id,
            RoiFovConfigs roiFovConfigs,
            RangingMode rangingMode,
            TimingBudget timingBudget) 
        {
            this(
                name.orElse("id " + id.getDeviceNumber()), 
                id, 
                new ProximityParamsConfigs(),
                roiFovConfigs, 
                rangingMode,
                timingBudget);
        }
    }

    @Getter
    private final String name;

    @Getter
    private final DistanceSensorConfiguration configuration;

    private LaserCanInterface distanceSensor = null;
    private int tries = 0;
    private boolean hasConfiged = false;

    private Distance currentDistance;

    private final Alert failedConfig =
        new Alert("Failed to configure LaserCAN!", AlertType.kError);
    private final Alert sensorAlert =
        new Alert("Failed to get LaserCAN measurement", Alert.AlertType.kWarning);

    /**
     * Constructor for DistanceSensorIOLaserCAN that initializes the LaserCAN, its status signals, and applies
     * configurations for the ToF Field of view and proximity detection.
     *
     * @param builder The instance used to build the DistanceSensorIOCANrange object.
     */
    public DistanceSensorIOLaserCAN(DistanceSensorConfiguration config, boolean isSim)
    {
        configuration = config;
        name = config.name();

        distanceSensor = getLaserCanInterface(config);

    }

    /** Gets the LC interface */
    public LaserCanInterface getLaserCanInterface(DistanceSensorConfiguration config) {
        if (distanceSensor != null) {
            return distanceSensor;
        } else {
            LaserCanInterface lc;
            lc = (Constants.currentMode == Constants.Mode.SIM)
                ? new LaserCANSim(name)
                : new LaserCan(config.id().getDeviceNumber());
            RegionOfInterest roi = new RegionOfInterest(
                (int) Math.round(config.roiFovConfigs().centerX()),
                (int) Math.round(config.roiFovConfigs().centerY()),
                (int) Math.round(config.roiFovConfigs().width()),
                (int) Math.round(config.roiFovConfigs().length()));
            while (!hasConfiged && tries < 5) {
                try {
                    lc.setRangingMode(config.rangingMode());
                    lc.setRegionOfInterest(roi);
                    lc.setTimingBudget(config.timingBudget());
                    failedConfig.set(false);
                    System.out.println("Succesfully configured " + name);
                    hasConfiged = true;
                } catch (ConfigurationFailedException e) {
                    System.out.println("Configuration failed for " + name + "! " + e);
                    failedConfig.setText("Failed to configure " + name + "!");
                    failedConfig.set(true);
                    tries++;
                }
            }
            return lc;
        }
    }

    /**
     * Updates the DistanceSensor inputs for tracking position, velocity, and other control parameters.
     *
     * @param inputs The DistanceSensorIOInputs object where the updated values will be stored.
     */
    @Override
    public void updateInputs(DistanceSensorIOInputs inputs)
    {
        Measurement measurement = distanceSensor.getMeasurement();
        if (measurement != null) {
            if (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
                sensorAlert.set(false);
                currentDistance = Millimeters.of(measurement.distance_mm);
                inputs.ambientSignal = (double) measurement.ambient;
                // The Region of interest:
                // x and y are the center point
                // w and h are the width and height of the region's rectangle
                inputs.roiX = measurement.roi.x;
                inputs.roiY = measurement.roi.y;
                inputs.roiW = measurement.roi.w;
                inputs.roiH = measurement.roi.h;
                inputs.isShortMode = !measurement.is_long;

            } else {
                sensorAlert.setText("Failed to get LaserCAN ID: " + name
                    + ", no valid measurement");
                sensorAlert.set(true);
                currentDistance = Millimeters.of(Double.POSITIVE_INFINITY);
                inputs.connected = false;
            }
        } else {
            sensorAlert.setText("Failed to get LaserCAN ID: " + name
                + ", measurement null");
            sensorAlert.set(true);
            currentDistance = Millimeters.of(Double.POSITIVE_INFINITY);
        }
        Logger.recordOutput("LaserCANSensors/LaserCAN" + name,
            currentDistance.in(Inches));
        inputs.distance = currentDistance;
    }
    
    /**
     * Updates the current settings that determine whether the CANrange 'detects' an object.
     * Call this method to achieve the desired performance.
     *
     * @param mode The LONG Ranging Mode can be used to identify targets at longer distances
     * than the short ranging mode (up to 4m), but is more susceptible to ambient
     * light.
     * The SHORT Ranging Mode is used to detect targets at 1.3m and lower. Although 
     * shorter than the Long ranging mode, this mode is less susceptible to ambient
     * light.
    * @throws ConfigurationFailedException 
    */
    private void setRangingMode(RangingMode mode)
    {
        hasConfiged = false;
        tries = 0;
        while (!hasConfiged && tries < 5) {
            try {
                distanceSensor.setRangingMode(mode);
                hasConfiged = true;
            } catch (ConfigurationFailedException e) {
                System.out.println("Configuration failed for " + name + "! " + e);
                failedConfig.setText("Failed to configure " + name + " new ranging mode!");
                failedConfig.set(true);
                tries++;
            }
        }
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
    private void setFOVParams(int newFOVCenterX, int newFOVCenterY, int newFOVRangeX, int newFOVRangeY)
    {
        RegionOfInterest config = new RegionOfInterest(newFOVCenterX, newFOVCenterY, newFOVRangeX, newFOVRangeY);
        hasConfiged = false;
        tries = 0;
        while (!hasConfiged && tries < 5) {
            try {
                distanceSensor.setRegionOfInterest(config);
                hasConfiged = true;
            } catch (ConfigurationFailedException e) {
                System.out.println("Configuration failed for " + name + "! " + e);
                failedConfig.setText("Failed to configure " + name + " new FOV params!");
                failedConfig.set(true);
                tries++;
            }
        }
    }

}