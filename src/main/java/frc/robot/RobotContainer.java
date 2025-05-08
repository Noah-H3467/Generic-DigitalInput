// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import frc.robot.genericSubsystems.GenericDigitalInput.DigitalInputIOWPILib;
import frc.robot.genericSubsystems.GenericDigitalInput.DigitalInputIOWPILib.DigitalInputIOConfiguration;
import frc.robot.genericSubsystems.GenericDigitalInput.DigitalInput;
import frc.robot.util.WindupXboxController;

/**
 * Container class for the entire robot structure. Handles subsystem instantiations, input devices,
 * and command bindings.
 */
public class RobotContainer {

    /**
    * Beambreak constants
    */
    private static final DigitalInputIOWPILib.DigitalInputIOConfiguration m_beamBreakConstants = 
        new DigitalInputIOConfiguration(
            Optional.of("Beam_Break"),
            0);
        
    // Initialize beambreak
    DigitalInput m_beamBreak = new DigitalInput(new DigitalInputIOWPILib(m_beamBreakConstants));

    private final WindupXboxController joystick = new WindupXboxController(0);

    /** Constructs the RobotContainer. Initializes all hardware and simulation IO. */
    public RobotContainer()
    {
        
        configureBindings();
    }

    private void configureBindings() {

        // Rumble when left trigger is pressed until the beam break is triggered
        joystick.leftTrigger().whileTrue(
            joystick.rumbleUntilCondition(0.5, () -> m_beamBreak.get()));
    }
    
}