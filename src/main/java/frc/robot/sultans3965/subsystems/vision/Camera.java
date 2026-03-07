package frc.robot.sultans3965.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.List;
import java.util.Optional;

public class Camera {
    private final PhotonCamera camera;
    private final Transform3d robotToCam;
    private final PhotonPoseEstimator poseEstimator;

    public Camera(AprilTagFieldLayout fieldLayout, String cameraNetworkTable, Transform3d robotToCam) {
        this.camera = new PhotonCamera(cameraNetworkTable);
        this.robotToCam = robotToCam;
        this.poseEstimator = new PhotonPoseEstimator(fieldLayout, robotToCam);
    }

    public void estimate() {
        List<PhotonPipelineResult> pipelineResults = unreadVisionResults();
        pipelineResults.parallelStream().forEach(p -> {
            Optional<EstimatedRobotPose> estimatedPose;
            synchronized (poseEstimator) {
                estimatedPose = poseEstimator.estimateCoprocMultiTagPose(p).or(() -> poseEstimator.estimateLowestAmbiguityPose(p));
            }
            estimatedPose.ifPresent(pose -> {

            });
        });
    }

    public List<PhotonPipelineResult> unreadVisionResults() {
        return camera.getAllUnreadResults();
    }

    public Transform3d transform() {
        return robotToCam;
    }
}
