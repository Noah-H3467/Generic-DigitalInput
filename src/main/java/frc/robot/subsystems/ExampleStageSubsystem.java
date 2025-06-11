package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Ports;
import frc.robot.GenericHardware.GenericDistanceSensor.DistanceSensor;
import frc.robot.GenericHardware.GenericDistanceSensor.DistanceSensorIO.RoiFovConfigs;
import frc.robot.GenericHardware.GenericDistanceSensor.DistanceSensorIOCANrange;
import frc.robot.GenericHardware.GenericDistanceSensor.DistanceSensorIOCANrange.DistanceSensorIOConfiguration;

import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

// Modified pre-AK version of Simple Subsystem but with a CANrange sensor to detect the game piece
public class ExampleStageSubsystem extends SubsystemBase {
    
    @RequiredArgsConstructor
    @Getter
    public enum State {
        ON(1.0),
        OFF(0.0);

        private final double output;
    }

    @Getter
    @Setter
    private State state = State.OFF;

    private boolean debug = true;

    TalonFX m_motor = new TalonFX(Ports.EXAMPLE_STAGE_PORT.getDeviceNumber());
    private final DutyCycleOut m_percent = new DutyCycleOut(0);
    private final NeutralOut m_neutral = new NeutralOut();

    DistanceSensorIOConfiguration config = new DistanceSensorIOConfiguration(
        Optional.of("Example Stage CANrange"),
        Ports.EXAMPLE_STAGE_CANRANGE,
        new RoiFovConfigs(0, 0, 0, 0),
        new ProximityParamsConfigs()
            .withMinSignalStrengthForValidMeasurement(2500)
            .withProximityHysteresis(0.02)
            .withProximityThreshold(0.17),
        RangingMode.SHORT,
        TimingBudget.TIMING_BUDGET_20MS);
    DistanceSensor canRange = new DistanceSensor(new DistanceSensorIOCANrange(config));

    /** Creates a new SimpleSubsystem. */
    public ExampleStageSubsystem() {
        m_motor.getConfigurator().apply(new TalonFXConfiguration());

    }

    @Override
    public void periodic() {

        if (state == State.OFF) {
            m_motor.setControl(m_neutral);
        } else {
            m_motor.setControl(m_percent.withOutput(state.getOutput()));
        }

        displayInfo(debug);
    }

    public boolean isTriggered() {
        return canRange.isDetected();
    }

    public Command setStateCommand(State state) {
        return runOnce(() -> this.state = state);
    }

    private void displayInfo(boolean debug) {
        if (debug) {
            SmartDashboard.putString(this.getClass().getSimpleName() + " State ", state.toString());
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Setpoint ", state.getOutput());
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Output ", m_motor.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Current Draw", m_motor.getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putBoolean(this.getClass().getSimpleName() + " Note in Stage? ", canRange.isDetected());
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Distance to Note (cm)", canRange.getDistance().magnitude());
        }
    }
}
