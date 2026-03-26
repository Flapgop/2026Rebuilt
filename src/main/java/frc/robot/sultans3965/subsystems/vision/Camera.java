package frc.robot.sultans3965.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sultans3965.constants.VisionConstants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

public class Camera extends SubsystemBase {
    private final PhotonCamera camera;
    private final Transform3d robotToCam;
    private final PhotonPoseEstimator poseEstimator;

    private Matrix<N3, N1> curStdDevs;
    private final VisionSubsystem.EstimateConsumer estConsumer;

    public Camera(AprilTagFieldLayout fieldLayout, String cameraNetworkTable, Transform3d robotToCam, VisionSubsystem.EstimateConsumer estConsumer) {
        this.camera = new PhotonCamera(cameraNetworkTable);
        this.robotToCam = robotToCam;
        this.poseEstimator = new PhotonPoseEstimator(fieldLayout, robotToCam);
        this.estConsumer = estConsumer;
        this.curStdDevs = VisionConstants.MULTI_TAG_STD_DEVS;

        System.out.printf("Camera created! %s\n", camera.getName());
    }

    public List<PhotonPipelineResult> unreadVisionResults() {
        return camera.getAllUnreadResults();
    }

    public Transform3d transform() {
        return robotToCam;
    }

    public Command estimate() {
        return run(() -> {
            List<PhotonPipelineResult> pipelineResults = unreadVisionResults();
            pipelineResults.stream().forEach(result -> {
                Optional<EstimatedRobotPose> estimatedPose;
                estimatedPose = poseEstimator.estimateCoprocMultiTagPose(result).or(() -> poseEstimator.estimateLowestAmbiguityPose(result));
                updateEstimationStdDevs(estimatedPose, result.getTargets());

                estimatedPose.ifPresent(est -> {
                    Matrix<N3, N1> estStdDevs = getEstimationStdDevs();
                    estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                });
            });
        });
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;
        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = VisionConstants.MULTI_TAG_STD_DEVS;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }
}
