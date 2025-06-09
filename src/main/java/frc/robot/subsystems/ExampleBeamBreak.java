package frc.robot.subsystems;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GenericHardware.GenericDigitalInput.DigitalInput;
import frc.robot.GenericHardware.GenericDigitalInput.DigitalInputIOWPILib;
import frc.robot.GenericHardware.GenericDigitalInput.DigitalInputIOWPILib.DigitalInputIOConfiguration;

public class ExampleBeamBreak extends SubsystemBase {
    
    public DigitalInput m_beamBreak;
    private String m_name;

    public ExampleBeamBreak(String name, int portNumber) {

        m_name = name;

        /**
        * Beambreak constants
        */
        final DigitalInputIOWPILib.DigitalInputIOConfiguration m_Constants = 
            new DigitalInputIOConfiguration(
                Optional.of(name),
                portNumber);
        
        // Initialize beambreak
        m_beamBreak = new DigitalInput(new DigitalInputIOWPILib(m_Constants));
    }

    @Override
    public void periodic() {
        displayInfo();
    }

    public boolean isTriggered() {
        return !m_beamBreak.get();
    }

    private void displayInfo() {
        SmartDashboard.putBoolean(m_name + "/Triggered", !m_beamBreak.get());
        Logger.recordOutput(m_name + "/Triggered", !m_beamBreak.get());
    }
}
