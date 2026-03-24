package frc.robot.sultans3965.subsystems.vision;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    @FunctionalInterface
    public interface EstimateConsumer {
        void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }

    private final Camera camera0, camera1;//, camera2, camera3;

    public VisionSubsystem(AprilTagFieldLayout layout, EstimateConsumer poseEstimateConsumer) {
        PortForwarder.add(5800, "photonvision.local", 5800);
        camera0 = new Camera(
            layout, 
            "photoncamera0", 
            new Transform3d(
                Distance.ofBaseUnits(-13.25, Inches), 
                Distance.ofBaseUnits(9.5, Inches), 
                Distance.ofBaseUnits(8.25, Inches), 
                new Rotation3d(
                    Angle.ofBaseUnits(0, Degree), 
                    Angle.ofBaseUnits(45.2, Degree), 
                    Angle.ofBaseUnits(0, Degree)
                )
            ),
            poseEstimateConsumer
        );
        camera1 = new Camera(
            layout, 
            "photoncamera1", 
            new Transform3d(
                Distance.ofBaseUnits(-13.25, Inches), 
                Distance.ofBaseUnits(-9.5, Inches), 
                Distance.ofBaseUnits(8.25, Inches), 
                new Rotation3d(
                    Angle.ofBaseUnits(0, Degree), 
                    Angle.ofBaseUnits(45.2, Degree), 
                    Angle.ofBaseUnits(0, Degree)
                )
            ), 
            poseEstimateConsumer
        );
        // camera2 = new Camera(layout, "photoncamera2", new Transform3d(), poseEstimateConsumer);
        // camera3 = new Camera(layout, "photoncamera3", new Transform3d(), poseEstimateConsumer);
    }

    public Command estimationLoop() {
        Command command = camera0.estimate().alongWith(camera1.estimate());//.alongWith(camera2.estimate()).alongWith(camera3.estimate());
        command.addRequirements(this);
        return command;
    }
}
