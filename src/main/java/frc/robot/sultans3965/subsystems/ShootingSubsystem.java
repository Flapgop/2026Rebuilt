package frc.robot.sultans3965.subsystems;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sultans3965.constants.Constants;

public class ShootingSubsystem extends SubsystemBase {

    private final SparkMax shooterIntake;
    private final SparkMax shooter;
    private final SparkMax intakeMovement;
    private final SparkMax intake;

    private final DigitalInput lowerLimit = new DigitalInput(0);
    private final DigitalInput upperLimit = new DigitalInput(1);

    private final SparkClosedLoopController shooterPID;

    private static final double MAX_RPM = 5676.0;
    private static final double MOTOR_KV = 473.0;
    private static final double WHEEL_DIAMETER_METERS = 0.0762; // ~3in?
    private static final double SHOOTER_ANGLE_RADIANS = Math.toRadians(80.0); // technically straight up but the stiction from the shooter compliant wheels forces the fuel to move in a specific direction
    private static final double SIN_SHOOTER_ANGLE = Math.sin(SHOOTER_ANGLE_RADIANS); 
    private static final double COS_SHOOTER_ANGLE = Math.cos(SHOOTER_ANGLE_RADIANS); 
    private static final double COMPRESSION_EFFICIENCY = 0.8;
    private static final double SHOOTER_RELEASE_HEIGHT_METERS = 0.5588;
    private static final double TARGET_HEIGHT = 1.83;

    // TODO: calculate fuel velocity. Something something, the exit velocity of a fuel is half the tangential velocity of the wheel, so like:
    // TODO: 0.5 * (RPM / 60 * pi * wheel diameter) * whatever fancy compression physics
    // TODO: then we feed that into a nice algorithm that determines where the ball might go

    public ShootingSubsystem() {
        shooterIntake = new SparkMax(14, MotorType.kBrushless);
        shooterIntake.configure(new SparkMaxConfig().idleMode(IdleMode.kCoast), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        shooter = new SparkMax(10, MotorType.kBrushless);
        intake = new SparkMax(11, MotorType.kBrushless);
        intake.configure(new SparkMaxConfig(), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        intakeMovement = new SparkMax(12, MotorType.kBrushless);
        intakeMovement.configure(new SparkMaxConfig().smartCurrentLimit(10), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        shooterPID = shooter.getClosedLoopController();
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);
        config.closedLoop
            .pid(0.0001, 0, 0);
        
        config.closedLoop.feedForward
            .kV(1.0 / MOTOR_KV)
            .kS(0.1); // TODO: tune this, static voltage needed to make the wheel break friction
        
        shooter.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        double currentRPM = shooter.getEncoder().getVelocity();
        double targetHeight = TARGET_HEIGHT - SHOOTER_RELEASE_HEIGHT_METERS;

        double v = 0.5 * ((currentRPM / 60.0) * Math.PI * WHEEL_DIAMETER_METERS) * COMPRESSION_EFFICIENCY;

        double vy = v * SIN_SHOOTER_ANGLE;
        double vx = v * COS_SHOOTER_ANGLE;

        double discriminant = (vy * vy) - (2 * Constants.GRAVITY * targetHeight);
        
        double estDistance = 0;
        if (discriminant >= 0) {
            double timeToTarget = (vy + Math.sqrt(discriminant)) / Constants.GRAVITY;
            estDistance = vx * timeToTarget;
        }

        SmartDashboard.putNumber("Shooter/Actual RPM", currentRPM);
        SmartDashboard.putNumber("Shooter/Est Distance (meters)", estDistance);
        SmartDashboard.putBoolean("Shooter/Can Reach Hub Height", discriminant >= 0);
    }

    public Command everything(Supplier<Double> intakeMovementSupplier, Supplier<Double> shooterSupplier, Supplier<Double> shooterIntakeSupplier, Supplier<Double> intakeSupplier) {
        return run(() -> {
            double moveSpeed = intakeMovementSupplier.get();

            if (moveSpeed < 0 && !lowerLimit.get()) moveSpeed = 0;
            if (moveSpeed > 0 && !upperLimit.get()) moveSpeed = 0;

            intakeMovement.set(moveSpeed);
            intake.set(intakeSupplier.get());
            shooterIntake.set(shooterIntakeSupplier.get());

            shooterPID.setSetpoint(shooterSupplier.get() * MAX_RPM, SparkMax.ControlType.kVelocity);
        });
    }
}
