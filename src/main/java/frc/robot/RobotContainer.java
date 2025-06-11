// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.GenericHardware.GenericDigitalInput.DigitalInputIOWPILib;
import frc.robot.GenericHardware.GenericDigitalInput.DigitalInputIOWPILib.DigitalInputIOConfiguration;
import frc.robot.subsystems.ExampleBeamBreak;
import frc.robot.subsystems.ExampleStageSubsystem;
import frc.robot.GenericHardware.GenericDigitalInput.DigitalInput;
import frc.robot.util.WindupXboxController;

/**
 * Container class for the entire robot structure. Handles subsystem instantiations, input devices,
 * and command bindings.
 */
public class RobotContainer {

    /**
     * There are two possible ways to implement the beambreak (or limit switch)
     * Option 1: instantiate it inside the RobotContainer, and get its value to use in commands
     * Option 2: instantiate it inside a subsystem, and use the subsystem to get its value.
     * With option 2, its value will be logged periodically as well as reported to the dashboard.
     */

    /**
    * Option 1: Beambreak constants
    */
    private static final DigitalInputIOWPILib.DigitalInputIOConfiguration m_beamBreakConstants = 
        new DigitalInputIOConfiguration(
            Optional.of("Beam_Break"),
            Ports.BEAM_BREAK_PORT);
        
    // Option 1: Initialize beambreak
    DigitalInput m_beamBreak = new DigitalInput(new DigitalInputIOWPILib(m_beamBreakConstants));

    // Option 2: Initialize example beambreak subsystem
    ExampleBeamBreak  m_beamBreakSub = new ExampleBeamBreak("Example_Beam_Break", Ports.BEAM_BREAK_SUB_PORT);

    // Instantiate example stage subsystem with CANrange
    ExampleStageSubsystem m_exampleStage = new ExampleStageSubsystem();

    private final WindupXboxController joystick = new WindupXboxController(0);

    /** Constructs the RobotContainer. Initializes all hardware and simulation IO. */
    public RobotContainer()
    {
        
        configureBindings();
    }

    private void configureBindings() {

        // Option 1: Rumble when left trigger is pressed until the beam break is triggered
        joystick.leftTrigger().and(joystick.rightTrigger().negate()).whileTrue(
            joystick.rumbleUntilCondition(0.5, () -> !m_beamBreak.get()));

        // Option 2: Rumble when right trigger is pressed until the beam break is triggered
        joystick.rightTrigger().and(joystick.leftTrigger().negate()).whileTrue(
            joystick.rumbleUntilCondition(0.5, () -> m_beamBreakSub.isTriggered()));

        // Example stage subsystem command
        joystick.leftBumper().onTrue(m_exampleStage.setStateCommand(ExampleStageSubsystem.State.ON)
            .andThen(Commands.waitUntil(() -> m_exampleStage.isTriggered())
            .andThen(m_exampleStage.setStateCommand(ExampleStageSubsystem.State.OFF))));

    }
    
}