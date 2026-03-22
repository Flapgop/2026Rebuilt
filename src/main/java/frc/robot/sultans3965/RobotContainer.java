// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sultans3965;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
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
                    () -> driver.getLeftX() * -1)
            .withControllerRotationAxis(driver::getRightX)
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.6)
            .allianceRelativeControl(true);

//     private final SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(swerveSubsystem.swerveDrive(),
//                     () -> -driver.getLeftY(),
//                     () -> -driver.getLeftX())
//             .withControllerRotationAxis(() -> driver.getRawAxis(2))
//             .deadband(OperatorConstants.DEADBAND)
//             .scaleTranslation(0.8)
//             .allianceRelativeControl(true);

//     private final SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
//             .withControllerHeadingAxis(
//                     () -> Math.sin(driver.getRawAxis(2) * Math.PI) * (Math.PI * 2),
//                     () -> Math.cos(driver.getRawAxis(2) * Math.PI) * (Math.PI * 2)
//             )
//             .headingWhile(true)
//             .translationHeadingOffset(true)
//             .translationHeadingOffset(Rotation2d.fromDegrees(
//                     0));

    private final VisionSubsystem visionSubsystem;
    private final ShootingSubsystem shootingSubsystem = new ShootingSubsystem();

    public RobotContainer(Robot robot) {
        visionSubsystem = new VisionSubsystem(AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField), swerveSubsystem::addVisionMeasurement);
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);
        NamedCommands.registerCommand("test", Commands.print("pong!"));
    }
    
    
    private void configureBindings() {
        Command driveFieldOrientedAngularVelocity = swerveSubsystem.driveFieldOriented(driveAngularVelocity);
        swerveSubsystem.resetOdometry(new Pose2d());
        // Command driveFieldOrientedAngularVelocityKeyboard = swerveSubsystem.driveFieldOriented(driveAngularVelocityKeyboard);
        Command visionEstimation = visionSubsystem.estimationLoop();
        // if (!RobotBase.isSimulation()) { // TODO: simulation
            swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);
        //     Pose2d target = new Pose2d(new Translation2d(1, 4),
        //             Rotation2d.fromDegrees(90));
        //     driveDirectAngleKeyboard.driveToPose(() -> target,
        //             new ProfiledPIDController(5,
        //                     0,
        //                     0,
        //                     new TrapezoidProfile.Constraints(5, 2)),
        //             new ProfiledPIDController(5,
        //                     0,
        //                     0,
        //                     new TrapezoidProfile.Constraints(Units.degreesToRadians(360),
        //                             Units.degreesToRadians(180))
        //             ));
            driver.start().onTrue(Commands.runOnce(() -> swerveSubsystem.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
        //     driver.button(1).whileTrue(swerveSubsystem.sysIdDriveMotorCommand());
        //     driver.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
        //             () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));
        // } else {
        //     swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocityKeyboard);
        // }
        visionSubsystem.setDefaultCommand(visionEstimation);

        driver.a().onTrue((Commands.runOnce(swerveSubsystem::zeroGyro)));
        driver.x().onTrue(Commands.runOnce(swerveSubsystem::addFakeVisionReading));
        driver.start().whileTrue(Commands.none());
        driver.back().whileTrue(Commands.none());
        driver.leftBumper().whileTrue(Commands.runOnce(swerveSubsystem::lock, swerveSubsystem).repeatedly());
        driver.rightBumper().onTrue(Commands.none());

        shootingSubsystem.setDefaultCommand(shootingSubsystem.everything(
                () -> operator.getLeftY(), 
                () -> -operator.getRightTriggerAxis(), 
                () -> Math.abs(operator.getRightTriggerAxis()) > 0.01 ? -0.8 : 0.0, 
                () -> operator.leftBumper().getAsBoolean() ? -0.7 : 0.0));
    }
    
    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Example Auto");
    }
}
