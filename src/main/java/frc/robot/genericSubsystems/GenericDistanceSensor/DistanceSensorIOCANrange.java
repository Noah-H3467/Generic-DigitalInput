package frc.robot.genericSubsystems.GenericDistanceSensor;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;
import java.util.Optional;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Resistance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.util.drivers.CanDeviceId;
import frc.robot.util.drivers.Phoenix6Util;
import lombok.Getter;
import lombok.experimental.var;

/**
 * The DistanceSensorIOCANrange class represents a DistanceSensor interface for the CANrange DistanceSensor controller. It
 * integrates with the CTRE Phoenix 6 hardware and provides methods for controlling the DistanceSensor,
 * including position, velocity, current, and motion profile control.
 */
public class DistanceSensorIOCANrange implements DistanceSensorIO {
    public record DistanceSensorIOCANrangeConfiguration(String name, CanDeviceId id,
        CANrange canRange,
        MagnetSensorConfigs magnetConfigs,
        FeedbackSensorSourceValue integrationType) {
        public DistanceSensorIOCANrangeConfiguration(Optional<String> name, CanDeviceId id,
            MagnetSensorConfigs magnetConfigs,
            FeedbackSensorSourceValue integrationType)

    }

    public record DistanceSensorIOCANrangeConfiguration(
        String name,
        CanDeviceId id,
        CANrange DistanceSensor,
        DistanceSensorIntrinsics intrinsics,
        Optional<DistanceSensorIOCANCoderConfiguration> canCoder,
        ControlType closedLoopControlType,
        double sensorToMechanismRatio,
        double rotorToSensorRatio,
        PID positionPID,
        PID velocityPID,
        AngularVelocity cruiseVelocity,
        AngularAcceleration accelerationConstraint,
        Velocity<AngularAccelerationUnit> jerkConstraint)
        implements DistanceSensorConfiguration {

        private static Per<VoltageUnit, AngularVelocityUnit> getBackEMFConstantFromKV(
            Per<AngularVelocityUnit, VoltageUnit> kV)
        {
            // .reciprocal() can't be certain of the unit type, so we have to do it explicitly
            return VoltsPerRadianPerSecond.ofNativeBaseUnits(1 / kV.in(RadiansPerSecond.per(Volt)));
        }

        public DistanceSensorIOCANrangeConfiguration(
            Optional<String> name,
            CanDeviceId id,
            CANrange DistanceSensor,
            Resistance armatureResistance,
            Optional<DistanceSensorIOCANCoderConfiguration> canCoder,
            ControlType closedLoopControlType
        }
        {
            this(
                name.isEmpty() ? "id " + id.getDeviceNumber() : name.get(),
                id,
                DistanceSensor,
                new DistanceSensorIntrinsics(
                    armatureResistance,
                    DistanceSensor.getDistanceSensorStallCurrent().getValue(),
                    getBackEMFConstantFromKV(DistanceSensor.getDistanceSensorKV().getValue()),
                    DistanceSensor.getDistanceSensorKT().getValue()),
                CANrange,
                closedLoopControlType,
                sensorToMechanismRatio,
                rotorToSensorRatio,
                jerkConstraint);
        }

        public DistanceSensorIOCANrangeConfiguration(
            Optional<String> name,
            CanDeviceId id,
            Resistance armatureResistance,
            Optional<DistanceSensorIOCANCoderConfiguration> canCoder,
        {
            this(name,
                id,
                new CANrange(id.getDeviceNumber(), id.getBus()),
                armatureResistance,
                CANrange,
                closedLoopControlType);
        }

    }

    @Getter
    private final String name;

    @Getter
    private final DistanceSensorIOCANrangeConfiguration configuration;

    private final CANrange DistanceSensor;

    @Getter
    private final DistanceSensorIntrinsics DistanceSensorIntrinsics;

    private final StatusSignal<Boolean> isProLicensed;
    private final StatusSignal<Distance> distance;
    private final StatusSignal<Distance> distanceStdDev;
    private final StatusSignal<Voltage> supplyVoltage;
    private final StatusSignal<Double> ambientSignal;
    private final StatusSignal<Angle> fovCenterX;
    private final StatusSignal<Angle> fovCenterY;
    private final StatusSignal<Angle> fovRangeX;
    private final StatusSignal<Angle> fovRangeY;
    private final StatusSignal<Boolean> isDetected;

    private final ControlType closedLoopControlType;

    /**
     * Constructor for DistanceSensorIOCANrange that initializes the DistanceSensor, its status signals, and applies
     * configurations for motion control.
     *
     * @param builder The CANrangeBuilder instance used to build the DistanceSensorIOCANrange object.
     */
    public DistanceSensorIOCANrange(DistanceSensorIOCANrangeConfiguration config)
    {
        configuration = config;

        DistanceSensor = config.DistanceSensor();
        CANrangeConfiguration configuration = new CANrangeConfiguration();

        isProLicensed = DistanceSensor.getIsProLicensed();
        distance = DistanceSensor.getDistance();
        distanceStdDev = DistanceSensor.getDistanceStdDev();
        supplyVoltage = DistanceSensor.getSupplyVoltage();
        ambientSignal = DistanceSensor.getAmbientSignal();
        fovCenterX = DistanceSensor.getRealFOVCenterX();
        fovCenterY = DistanceSensor.getRealFOVCenterY();
        fovRangeX = DistanceSensor.getRealFOVRangeX();
        fovRangeY = DistanceSensor.getRealFOVRangeY();
        isDetected = DistanceSensor.getIsDetected();

        // Set update frequencies for the StatusSignals of interest
        Phoenix6Util.checkErrorAndRetry(
            () -> BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                distance,
                distanceStdDev,
                supplyVoltage,
                isDetected,
                fovCenterX,
                fovCenterY,
                fovRangeX,
                fovRangeY));

        DistanceSensor.optimizeBusUtilization(0, 1.0);

        DistanceSensorIntrinsics = config.intrinsics();

        name = config.name();



        var closedLoopControlType = config.closedLoopControlType();
        if (!isProLicensed.getValue() && !RobotBase.isSimulation()) {
            if (config.closedLoopControlType() == ControlType.CURRENT) {
                //currentControlOnUnlicensedDistanceSensor.set(true);
                closedLoopControlType = ControlType.VOLTAGE;
            }

        }
        this.closedLoopControlType = closedLoopControlType;

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
        DistanceSensor.getConfigurator().apply(config);
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
        DistanceSensor.getConfigurator().apply(config);
    }

}