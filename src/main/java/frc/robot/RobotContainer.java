// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Mechanism;

public class RobotContainer {

    CommandXboxController m_joystick = new CommandXboxController(0);
    Mechanism m_mechanism = new Mechanism();

    public RobotContainer() {
        configureBindings();
    }

    private double getLeftY() {
        if (Math.abs(m_joystick.getLeftY()) > 0.1) {
            return -m_joystick.getLeftY();
        }
        return 0;
    }

    private void configureBindings() {
        /* Default command is duty cycle control with the left up/down stick */
        m_mechanism.setDefaultCommand(m_mechanism.joystickDriveCommand(this::getLeftY));

        /**
         * Joystick Y = quasistatic forward
         * Joystick B = dynamic forward
         * Joystick A = quasistatic reverse
         * Joystick X = dyanmic reverse
         */
        m_joystick.y().whileTrue(m_mechanism.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        m_joystick.a().whileTrue(m_mechanism.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        m_joystick.b().whileTrue(m_mechanism.sysIdDynamic(SysIdRoutine.Direction.kForward));
        m_joystick.x().whileTrue(m_mechanism.sysIdDynamic(SysIdRoutine.Direction.kReverse));


        /* Manually stop logging with left bumper after we're done with the tests */
        /* This isn't necessary, but is convenient to reduce the size of the hoot file */
        m_joystick.leftBumper().onTrue(new RunCommand(SignalLogger::stop));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
