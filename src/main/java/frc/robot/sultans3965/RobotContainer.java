// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sultans3965;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.sultans3965.constants.OperatorConstants;
import frc.robot.sultans3965.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import java.io.File;


public class RobotContainer
{
    private final CommandXboxController driver = new CommandXboxController(0);
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory() + File.separator + "swerve"));
    private final SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.swerveDrive(),
                    () -> driver.getLeftY() * -1,
                    () -> driver.getLeftX() * -1)
            .withControllerRotationAxis(driver::getRightX)
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);

    public RobotContainer(Robot robot)
    {
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);
        NamedCommands.registerCommand("test", Commands.print("pong!"));
    }
    
    
    private void configureBindings() {
        Command driveFieldOrientedAngularVelocity = swerveSubsystem.driveFieldOriented(driveAngularVelocity);
        if (!RobotBase.isSimulation()) { // TODO: simulation
            swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);
        }

        driver.a().onTrue((Commands.runOnce(swerveSubsystem::zeroGyro)));
        driver.x().onTrue(Commands.runOnce(swerveSubsystem::addFakeVisionReading));
        driver.start().whileTrue(Commands.none());
        driver.back().whileTrue(Commands.none());
        driver.leftBumper().whileTrue(Commands.runOnce(swerveSubsystem::lock, swerveSubsystem).repeatedly());
        driver.rightBumper().onTrue(Commands.none());
    }
    
    
    public Command getAutonomousCommand()
    {
        return Commands.print("No autonomous command configured");
    }
}
