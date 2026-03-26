// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sultans3965;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.sultans3965.constants.OperatorConstants;
import frc.robot.sultans3965.subsystems.ShootingSubsystem;
import frc.robot.sultans3965.subsystems.SwerveSubsystem;
import frc.robot.sultans3965.subsystems.vision.VisionSubsystem;
import swervelib.SwerveInputStream;

import java.io.File;


public class RobotContainer
{
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory() + File.separator + "swerve"));
    private final SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.swerveDrive(),
                    () -> driver.getLeftY(),
                    () -> driver.getLeftX())
            .withControllerRotationAxis(driver::getRightX)
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.6)
            .allianceRelativeControl(true);

    private final SendableChooser<Command> autoChooser;

    private final VisionSubsystem visionSubsystem;
    private final ShootingSubsystem shootingSubsystem = new ShootingSubsystem();

    public RobotContainer(Robot robot) {
        visionSubsystem = new VisionSubsystem(AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField), swerveSubsystem::addVisionMeasurement);
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Selection: ", autoChooser);
    }
    
    
    private void configureBindings() {
        Command driveFieldOrientedAngularVelocity = swerveSubsystem.driveFieldOriented(driveAngularVelocity);
        Command visionEstimation = visionSubsystem.estimationLoop();
        swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);
        visionSubsystem.setDefaultCommand(visionEstimation);

        driver.start().onTrue(Commands.runOnce(swerveSubsystem::zeroGyroWithAlliance));
        driver.back().onTrue(Commands.none());
        driver.leftBumper().whileTrue(Commands.runOnce(swerveSubsystem::lock, swerveSubsystem).repeatedly());
        driver.rightBumper().onTrue(Commands.none());

        shootingSubsystem.setDefaultCommand(shootingSubsystem.everything(
                () -> operator.getLeftY(), 
                () -> -operator.getRightTriggerAxis(), 
                () -> -operator.getLeftTriggerAxis(), 
                () -> operator.leftBumper().getAsBoolean() ? -0.7 : 0.0
        ));
    }
    
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}