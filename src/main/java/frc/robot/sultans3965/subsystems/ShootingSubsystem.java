package frc.robot.sultans3965.subsystems;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShootingSubsystem extends SubsystemBase {

    private final SparkMax shooterAgitator;
    private final SparkMax shooter;
    private final SparkMax intakeMovement;
    private final SparkMax intake;

    public ShootingSubsystem() {
        shooterAgitator = new SparkMax(14, MotorType.kBrushless);
        shooter = new SparkMax(10, MotorType.kBrushless);
        intake = new SparkMax(11, MotorType.kBrushless);
        intakeMovement = new SparkMax(12, MotorType.kBrushless);
    }

    public void shoot(double speed) {
        shooter.set(speed);
    }

    public Command shoot(Supplier<Double> shooterSpeedSupplier) {
        return run(() -> {
            
        });
    }

    public Command intake(Supplier<Double> intakeSpeedSupplier) {
        return run(() -> {
            intake.set(intakeSpeedSupplier.get());
        });
    }

    public Command moveIntake(Supplier<Double> movementSupplier) {
        return run(() -> {
            double blegh = movementSupplier.get();
            System.out.println("Setting movement to: " + blegh);
            intakeMovement.set(blegh);
        });
    }

    public Command agitate(Supplier<Double> agitationSupplier) {
        return run(() -> {
            shooterAgitator.set(agitationSupplier.get());
        });
    }
}
