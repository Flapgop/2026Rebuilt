package frc.robot.sultans3965.subsystems;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShootingSubsystem extends SubsystemBase {

    private final SparkMax shooterAgitator;
    private final SparkMax shooter;
    private final SparkMax intakeMovement;
    private final SparkMax intake;

    public ShootingSubsystem() {
        shooterAgitator = new SparkMax(14, MotorType.kBrushless);
        shooterAgitator.configure(new SparkMaxConfig().idleMode(IdleMode.kCoast), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        shooter = new SparkMax(10, MotorType.kBrushless);
        intake = new SparkMax(11, MotorType.kBrushless);
        intake.configure(new SparkMaxConfig(), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        intakeMovement = new SparkMax(12, MotorType.kBrushless);
        intakeMovement.configure(new SparkMaxConfig().smartCurrentLimit(10), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command everything(Supplier<Double> intakeMovementSupplier, Supplier<Double> shooterSupplier, Supplier<Double> agitatorSupplier, Supplier<Double> intakeSupplier) {
        return run(() -> {
            intakeMovement.set(intakeMovementSupplier.get());
            intake.set(intakeSupplier.get());
            shooterAgitator.set(agitatorSupplier.get());
            shooter.set(shooterSupplier.get());
        });
    }
}
