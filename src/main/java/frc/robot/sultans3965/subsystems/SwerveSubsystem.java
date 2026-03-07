package frc.robot.sultans3965.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.sultans3965.AllianceHelper;
import frc.robot.sultans3965.constants.Constants;
import org.json.simple.parser.ParseException;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.io.File;
import java.io.IOException;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Meter;

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveDrive swerveDrive;

    public SwerveSubsystem(File directory) {
        Pose2d startingPose = AllianceHelper.isBlueAlliance()
                ? new Pose2d(new Translation2d(Meter.of(1),
                Meter.of(4)),
                Rotation2d.fromDegrees(0))
                : new Pose2d(new Translation2d(Meter.of(16),
                Meter.of(4)),
                Rotation2d.fromDegrees(180));
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;

        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED, startingPose);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        swerveDrive.setHeadingCorrection(false);
        swerveDrive.setCosineCompensator(false);
        swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
        swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
        setupPathPlanner();
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(this::zeroGyroWithAlliance));
    }

    public void setupPathPlanner() {
        try {
            RobotConfig config = RobotConfig.fromGUISettings();
            final boolean enableFeedForward = false;
            AutoBuilder.configure(
                    this::pose,
                    this::resetOdometry,
                    this::robotVelocity,
                    (speedsRobotRelative, moduleFeedForwards) -> {
                        if (enableFeedForward) {
                            swerveDrive.drive(
                                    speedsRobotRelative,
                                    swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                                    moduleFeedForwards.linearForces()
                            );
                        } else {
                            swerveDrive.setChassisSpeeds(speedsRobotRelative);
                        }
                    },
                    new PPHolonomicDriveController(
                            new PIDConstants(5.0, 0.0, 0.0),
                            new PIDConstants(5.0, 0.0, 0.0)
                    ),
                    config,
                    AllianceHelper::isRedAlliance,
                    this
            );
        } catch (IOException | ParseException e) {
            throw new RuntimeException(e);
        }
    }

    public void lock() {
        swerveDrive.lockPose();
    }

    public void addFakeVisionReading() {
        swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
    }

    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(() ->
            swerveDrive.driveFieldOriented(velocity.get())
        );
    }

    public ChassisSpeeds fieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    public ChassisSpeeds robotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    public Pose2d pose() {
        return swerveDrive.getPose();
    }

    public Rotation2d heading() {
        return pose().getRotation();
    }

    public void motorBrake(boolean brake) {
        swerveDrive.setMotorIdleMode(brake);
    }

    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    public void zeroGyroWithAlliance() {
        if (AllianceHelper.isBlueAlliance()) {
            zeroGyro();
        } else {
            zeroGyro();
            resetOdometry(new Pose2d(pose().getTranslation(), Rotation2d.fromDegrees(180)));
        }
    }

    public SwerveDrive swerveDrive() {
        return swerveDrive;
    }
}
