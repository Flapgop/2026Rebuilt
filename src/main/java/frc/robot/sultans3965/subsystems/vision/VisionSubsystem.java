package frc.robot.sultans3965.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    private final Camera camera0, camera1, camera2, camera3;

    public VisionSubsystem(AprilTagFieldLayout layout) {
        PortForwarder.add(5800, "photonvision.local", 5800);
        camera0 = new Camera(layout, "photoncamera0", new Transform3d()); // TODO: get camera loc measurements
        camera1 = new Camera(layout, "photoncamera1", new Transform3d());
        camera2 = new Camera(layout, "photoncamera2", new Transform3d());
        camera3 = new Camera(layout, "photoncamera3", new Transform3d());

    }
}
