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
import com.ctre.phoenix6.configs.LaserCANConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.LaserCANConfiguration;
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
import com.ctre.phoenix6.hardware.LaserCAN;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
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

/**
 * The DistanceSensorIOLaserCAN class represents a DistanceSensor interface for the LaserCAN DistanceSensor controller. It
 * integrates with Thrifty's hardware and provides methods for controlling the DistanceSensor,
 * including position, velocity, current, and motion profile control.
 */
public class DistanceSensorIOLaserCAN implements DistanceSensorIO {
    public record DistanceSensorIOLaserCANConfiguration(String name, CanDeviceId id,
        LaserCAN LaserCAN,
        MagnetSensorConfigs magnetConfigs,
        FeedbackSensorSourceValue integrationType) {
        public DistanceSensorIOLaserCANConfiguration(Optional<String> name, CanDeviceId id,
            MagnetSensorConfigs magnetConfigs,
            FeedbackSensorSourceValue integrationType)
        {
            this(name.orElse("id " + id.getDeviceNumber()), id,
                new LaserCAN(id.getDeviceNumber(), id.getBus()), magnetConfigs,
                integrationType);
        }
    }

    public record DistanceSensorIOLaserCANConfiguration(
        String name,
        CanDeviceId id,
        LaserCAN DistanceSensor,
        DistanceSensorIntrinsics intrinsics,
        Optional<DistanceSensorIOLaserCANConfiguration> LaserCAN,
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

        public DistanceSensorIOLaserCANConfiguration(
            Optional<String> name,
            CanDeviceId id,
            LaserCAN DistanceSensor,
            Resistance armatureResistance,
            Optional<DistanceSensorIOLaserCANConfiguration> LaserCAN,
            ControlType closedLoopControlType,
            double sensorToMechanismRatio,
            double rotorToSensorRatio,
            PID positionPID,
            PID velocityPID,
            AngularVelocity cruiseVelocity,
            AngularAcceleration accelerationConstraint,
            Velocity<AngularAccelerationUnit> jerkConstraint)
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
                LaserCAN,
                closedLoopControlType,
                sensorToMechanismRatio,
                rotorToSensorRatio,
                positionPID, velocityPID,
                cruiseVelocity,
                accelerationConstraint,
                jerkConstraint);
        }

        public DistanceSensorIOLaserCANConfiguration(
            Optional<String> name,
            CanDeviceId id,
            Resistance armatureResistance,
            Optional<DistanceSensorIOLaserCANConfiguration> LaserCAN,
            ControlType closedLoopControlType,
            double sensorToMechanismRatio,
            double rotorToSensorRatio,
            PID positionPID,
            PID velocityPID,
            AngularVelocity cruiseVelocity,
            AngularAcceleration accelerationConstraint,
            Velocity<AngularAccelerationUnit> jerkConstraint)
        {
            this(name,
                id,
                new LaserCAN(id.getDeviceNumber(), id.getBus()),
                armatureResistance,
                LaserCAN,
                closedLoopControlType,
                sensorToMechanismRatio,
                rotorToSensorRatio,
                positionPID,
                velocityPID,
                cruiseVelocity,
                accelerationConstraint,
                jerkConstraint);
        }

        public DistanceSensorIOLaserCANConfiguration(
            Optional<String> name,
            CanDeviceId id,
            Resistance armatureResistance,
            Optional<DistanceSensorIOLaserCANConfiguration> LaserCAN)
        {
            this(
                name,
                id,
                armatureResistance,
                LaserCAN,
                ControlType.VOLTAGE,
                1,
                1,
                new PID(0, 0, 0),
                new PID(0, 0, 0),
                RotationsPerSecond.of(0.0),
                RotationsPerSecondPerSecond.of(0.0),
                RotationsPerSecondPerSecond.per(Second).of(0.0));
        }
    }

    @Getter
    private final String name;

    @Getter
    private final DistanceSensorIOLaserCANConfiguration configuration;

    private final LaserCAN DistanceSensor;

    @Getter
    private final DistanceSensorIntrinsics DistanceSensorIntrinsics;

    private final StatusSignal<Boolean> isProLicensed;
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> supplyVoltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> temperature;
    private final StatusSignal<Double> closedLoopError;
    private final StatusSignal<Double> closedLoopTarget;
    private final StatusSignal<Double> closedLoopTargetDerivative;

    private final ControlType closedLoopControlType;

    private final VoltageOut voltageOut =
        new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);
    private final TorqueCurrentFOC currentOut = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

    private final PositionTorqueCurrentFOC positionCurrent =
        new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
    private final VelocityTorqueCurrentFOC velocityCurrent =
        new VelocityTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

    private final PositionVoltage positionVoltage =
        new PositionVoltage(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);
    private final VelocityVoltage velocityVoltage =
        new VelocityVoltage(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);

    private final MotionMagicTorqueCurrentFOC motionMagicPositionCurrent =
        new MotionMagicTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
    private final MotionMagicVelocityTorqueCurrentFOC motionMagicVelocityCurrent =
        new MotionMagicVelocityTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

    private final MotionMagicVoltage motionMagicPositionVoltage =
        new MotionMagicVoltage(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);
    private final MotionMagicVelocityVoltage motionMagicVelocityVoltage =
        new MotionMagicVelocityVoltage(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);

    private final CoastOut coastOut = new CoastOut();
    private final StaticBrake staticBrake = new StaticBrake();

    private Double g = 0.0;

    private final Alert currentControlOnUnlicensedDistanceSensor;
    private final Alert fusedLaserCANOnUnlicensedDistanceSensor;
    private final Alert syncLaserCANOnUnlicensedDistanceSensor;

    private Alert LaserCANOnDifferentBus = null;

    /**
     * Constructor for DistanceSensorIOLaserCAN that initializes the DistanceSensor, its status signals, and applies
     * configurations for motion control.
     *
     * @param builder The LaserCANBuilder instance used to build the DistanceSensorIOLaserCAN object.
     */
    public DistanceSensorIOLaserCAN(DistanceSensorIOLaserCANConfiguration config)
    {
        configuration = config;

        DistanceSensor = config.DistanceSensor();
        LaserCANConfiguration configuration = new LaserCANConfiguration();

        isProLicensed = DistanceSensor.getIsProLicensed();
        position = DistanceSensor.getPosition();
        velocity = DistanceSensor.getVelocity();
        supplyVoltage = DistanceSensor.getDistanceSensorVoltage();
        supplyCurrent = DistanceSensor.getSupplyCurrent();
        torqueCurrent = DistanceSensor.getTorqueCurrent();
        temperature = DistanceSensor.getDeviceTemp();
        closedLoopError = DistanceSensor.getClosedLoopError();
        closedLoopTarget = DistanceSensor.getClosedLoopReference();
        closedLoopTargetDerivative = DistanceSensor.getClosedLoopReferenceSlope();

        // Set update frequencies for the StatusSignals of interest
        Phoenix6Util.checkErrorAndRetry(
            () -> BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                position,
                velocity,
                supplyCurrent,
                supplyCurrent,
                torqueCurrent,
                temperature));
        Phoenix6Util.checkErrorAndRetry(
            () -> BaseStatusSignal.setUpdateFrequencyForAll(
                200,
                closedLoopError,
                closedLoopTarget,
                closedLoopTargetDerivative));
        DistanceSensor.optimizeBusUtilization(0, 1.0);

        DistanceSensorIntrinsics = config.intrinsics();

        name = config.name();

        currentControlOnUnlicensedDistanceSensor =
            new Alert("Current control used on unlicensed DistanceSensor " + name, AlertType.kError);
        fusedLaserCANOnUnlicensedDistanceSensor =
            new Alert("FusedLaserCAN used on unlicensed DistanceSensor " + name, AlertType.kError);
        syncLaserCANOnUnlicensedDistanceSensor =
            new Alert("SyncLaserCAN used on unlicensed DistanceSensor " + name, AlertType.kError);

        var closedLoopControlType = config.closedLoopControlType();
        if (!isProLicensed.getValue() && !RobotBase.isSimulation()) {
            if (config.closedLoopControlType() == ControlType.CURRENT) {
                currentControlOnUnlicensedDistanceSensor.set(true);
                closedLoopControlType = ControlType.VOLTAGE;
            }

            if (config.LaserCAN().isPresent()) {
                if (config.LaserCAN().get()
                    .integrationType() == FeedbackSensorSourceValue.FusedLaserCAN) {
                    fusedLaserCANOnUnlicensedDistanceSensor.set(true);
                }
                if (config.LaserCAN().get()
                    .integrationType() == FeedbackSensorSourceValue.SyncLaserCAN) {
                    syncLaserCANOnUnlicensedDistanceSensor.set(true);
                }
            }
        }
        this.closedLoopControlType = closedLoopControlType;

        configuration.Slot0
            .withKP(config.positionPID().p())
            .withKI(config.positionPID().i())
            .withKD(config.positionPID().d());

        configuration.Slot1
            .withKP(config.velocityPID().p())
            .withKI(config.velocityPID().i())
            .withKD(config.velocityPID().d());

        configuration.MotionMagic
            .withMotionMagicCruiseVelocity(config.cruiseVelocity())
            .withMotionMagicAcceleration(config.accelerationConstraint())
            .withMotionMagicJerk(config.jerkConstraint());

        if (config.LaserCAN().isPresent()) {
            var LaserCANConfig = config.LaserCAN().get();
            LaserCANOnDifferentBus =
                new Alert("LaserCAN " + LaserCANConfig.name() + " is not on the same bus as DistanceSensor "
                    + name, AlertType.kError);

            if (!config.id().getBus().equals(LaserCANConfig.id().getBus())) {
                LaserCANOnDifferentBus.set(true);
            } else {
                var LaserCANDeviceConfig = new LaserCANConfiguration();
                LaserCANDeviceConfig.MagnetSensor = LaserCANConfig.magnetConfigs();
                Phoenix6Util.applyAndCheckConfiguration(LaserCANConfig.LaserCAN(),
                    LaserCANDeviceConfig);

                LaserCANConfig.LaserCAN().optimizeBusUtilization(0, 1.0);

                configuration.Feedback.FeedbackRemoteSensorID =
                    LaserCANConfig.id().getDeviceNumber();

                configuration.Feedback.FeedbackSensorSource = LaserCANConfig.integrationType();
                configuration.Feedback.RotorToSensorRatio = config.rotorToSensorRatio();
                configuration.Feedback.SensorToMechanismRatio = config.sensorToMechanismRatio();
            }
        }

        Phoenix6Util.applyAndCheckConfiguration(DistanceSensor, configuration);
    }

    private boolean isRunningPositionControl()
    {
        var control = DistanceSensor.getAppliedControl();
        return (control instanceof PositionTorqueCurrentFOC)
            || (control instanceof PositionVoltage)
            || (control instanceof MotionMagicTorqueCurrentFOC)
            || (control instanceof MotionMagicVoltage);
    }

    private boolean isRunningVelocityControl()
    {
        var control = DistanceSensor.getAppliedControl();
        return (control instanceof VelocityTorqueCurrentFOC)
            || (control instanceof VelocityVoltage)
            || (control instanceof MotionMagicVelocityTorqueCurrentFOC)
            || (control instanceof MotionMagicVelocityVoltage);
    }

    private boolean isRunningMotionMagic()
    {
        var control = DistanceSensor.getAppliedControl();
        return (control instanceof MotionMagicTorqueCurrentFOC)
            || (control instanceof MotionMagicVelocityTorqueCurrentFOC)
            || (control instanceof MotionMagicVoltage)
            || (control instanceof MotionMagicVelocityVoltage);
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
            position,
            velocity,
            supplyVoltage,
            supplyCurrent,
            torqueCurrent,
            temperature,
            closedLoopError,
            closedLoopTarget,
            closedLoopTargetDerivative)
            .isOK();

        // Updates the inputs with the DistanceSensor values
        inputs.position = position.getValue();
        inputs.velocity = velocity.getValue();
        inputs.appliedVoltage = supplyVoltage.getValue();
        inputs.supplyCurrent = supplyCurrent.getValue();
        inputs.torqueCurrent = torqueCurrent.getValue();
        inputs.temperature = temperature.getValue();

        // Updates position and velocity errors when relevant
        var closedLoopErrorValue = closedLoopError.getValue();
        var closedLoopTargetValue = closedLoopTarget.getValue();
        inputs.positionError = isRunningPositionControl()
            ? Rotations.of(closedLoopErrorValue)
            : null;
        inputs.activeTrajectoryPosition = isRunningPositionControl() && isRunningMotionMagic()
            ? Rotations.of(closedLoopTargetValue)
            : null;
        if (isRunningVelocityControl()) {
            inputs.velocityError = RotationsPerSecond.of(closedLoopErrorValue);
            inputs.activeTrajectoryVelocity =
                RotationsPerSecond.of(closedLoopTargetValue);
        } else if (isRunningPositionControl() && isRunningMotionMagic()) {
            var targetVelocity = closedLoopTargetDerivative.getValue();
            inputs.velocityError = RotationsPerSecond.of(
                targetVelocity - velocity.getValue().in(RotationsPerSecond));
            inputs.activeTrajectoryVelocity = RotationsPerSecond.of(targetVelocity);
        } else {
            inputs.velocityError = null;
            inputs.activeTrajectoryVelocity = null;
        }
    }

    /**
     * Sets the DistanceSensor to coast mode
     */
    @Override
    public void runCoast()
    {
        DistanceSensor.setControl(coastOut);
    }

    /**
     * Sets the DistanceSensor to brake mode
     */
    @Override
    public void runBrake()
    {
        DistanceSensor.setControl(staticBrake);
    }

    /**
     * Sets the DistanceSensor to run at a specified voltage.
     *
     * @param voltage The voltage to apply to the DistanceSensor.
     */
    @Override
    public void runVoltage(Voltage voltage)
    {
        currentControlOnUnlicensedDistanceSensor.set(false);
        DistanceSensor.setControl(voltageOut.withOutput(voltage));
    }

    /**
     * Sets the DistanceSensor to run at a specified current.
     *
     * @param current The current to apply to the DistanceSensor.
     */
    @Override
    public void runCurrent(Current current)
    {
        if (DistanceSensor.getIsProLicensed().getValue() && !RobotBase.isSimulation()) {
            DistanceSensor.setControl(currentOut.withOutput(current));
        } else {
            currentControlOnUnlicensedDistanceSensor.set(true);
        }
    }

    /**
     * Updates the current feed-forward for the DistanceSensor controls. This is used to adjust the DistanceSensor
     * behavior to achieve the desired performance.
     *
     * @param newFF The new feed-forward value to set for the DistanceSensor.
     */
    private void updateCurrentFeedForward(double newFF)
    {
        var prevControl = DistanceSensor.getAppliedControl();
        if (prevControl instanceof PositionTorqueCurrentFOC control) {
            control.withFeedForward(newFF);
        } else if (prevControl instanceof VelocityTorqueCurrentFOC control) {
            control.withFeedForward(newFF);
        } else if (prevControl instanceof PositionVoltage control) {
            control.withFeedForward(newFF);
        } else if (prevControl instanceof VelocityVoltage control) {
            control.withFeedForward(newFF);
        } else if (prevControl instanceof MotionMagicTorqueCurrentFOC control) {
            control.withFeedForward(newFF);
        } else if (prevControl instanceof MotionMagicVelocityTorqueCurrentFOC control) {
            control.withFeedForward(newFF);
        } else if (prevControl instanceof MotionMagicVoltage control) {
            control.withFeedForward(newFF);
        } else if (prevControl instanceof MotionMagicVelocityVoltage control) {
            control.withFeedForward(newFF);
        }
    }

    /**
     * Sets the feed-forward constant to adjust the DistanceSensor's behavior for position and velocity
     * control. This allows for better performance when operating in motion control modes.
     *
     * @param g The feed-forward value to set.
     */
    @Override
    public void setG(double g)
    {
        this.g = g;
        updateCurrentFeedForward(g);
    }

    /**
     * Commands the DistanceSensor to move to a specified position using position control.
     *
     * @param positionRotations The target position.
     */
    @Override
    public void runToPosition(Angle position)
    {
        if (closedLoopControlType == ControlType.CURRENT) {
            if (!DistanceSensor.getIsProLicensed().getValue() && !RobotBase.isSimulation()) {
                currentControlOnUnlicensedDistanceSensor.set(true);
                return;
            }
            DistanceSensor.setControl(positionCurrent.withPosition(position).withFeedForward(g).withSlot(0));
            currentControlOnUnlicensedDistanceSensor.set(false);
        } else {
            DistanceSensor.setControl(positionVoltage.withPosition(position).withFeedForward(g).withSlot(0));
        }
    }

    /**
     * Commands the DistanceSensor to run at a specified velocity using velocity control.
     *
     * @param velocityRPS The target velocity.
     */
    @Override
    public void runToVelocity(AngularVelocity velocity)
    {
        if (closedLoopControlType == ControlType.CURRENT) {
            if (!DistanceSensor.getIsProLicensed().getValue() && !RobotBase.isSimulation()) {
                currentControlOnUnlicensedDistanceSensor.set(true);
                return;
            }
            DistanceSensor.setControl(velocityCurrent.withFeedForward(g).withSlot(1));
            currentControlOnUnlicensedDistanceSensor.set(false);
        } else {
            DistanceSensor.setControl(velocityVoltage.withFeedForward(g).withSlot(1));
        }
    }

    /**
     * Commands the DistanceSensor to follow a motion profile to reach a target position.
     *
     * @param positionRotations The target position.
     */
    @Override
    public void runMotionProfiledPosition(Angle position)
    {
        if (closedLoopControlType == ControlType.CURRENT) {
            if (!DistanceSensor.getIsProLicensed().getValue() && !RobotBase.isSimulation()) {
                currentControlOnUnlicensedDistanceSensor.set(true);
                return;
            }
            DistanceSensor.setControl(
                motionMagicPositionCurrent.withPosition(position).withFeedForward(g).withSlot(0));
            currentControlOnUnlicensedDistanceSensor.set(false);
        } else {
            DistanceSensor.setControl(
                motionMagicPositionVoltage.withPosition(position).withFeedForward(g).withSlot(0));
        }
    }

    /**
     * Commands the DistanceSensor to follow a motion profile to reach a target velocity.
     *
     * @param velocityRPS The target velocity.
     */
    @Override
    public void runMotionProfiledVelocity(AngularVelocity velocity)
    {
        if (closedLoopControlType == ControlType.CURRENT) {
            if (!DistanceSensor.getIsProLicensed().getValue() && !RobotBase.isSimulation()) {
                currentControlOnUnlicensedDistanceSensor.set(true);
                return;
            }
            DistanceSensor.setControl(motionMagicVelocityCurrent.withFeedForward(g).withSlot(1));
            currentControlOnUnlicensedDistanceSensor.set(false);
        } else {
            DistanceSensor.setControl(motionMagicVelocityVoltage.withFeedForward(g).withSlot(1));
        }
    }

    @Override
    public void setPosition(Angle position)
    {
        DistanceSensor.setPosition(position);
    }
}
