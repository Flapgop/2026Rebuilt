package frc.robot.sultans3965.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sultans3965.Robot;
import frc.robot.sultans3965.constants.VisionConstants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

public class Camera extends SubsystemBase {
    private final PhotonCamera camera;
    private final Transform3d robotToCam;
    private final PhotonPoseEstimator poseEstimator;

    private final VisionSystemSim visionSim;
    private final PhotonCameraSim cameraSim;

    private Matrix<N3, N1> curStdDevs;
    private final VisionSubsystem.EstimateConsumer estConsumer;

    public Camera(AprilTagFieldLayout fieldLayout, String cameraNetworkTable, Transform3d robotToCam, VisionSubsystem.EstimateConsumer estConsumer) {
        this.camera = new PhotonCamera(cameraNetworkTable);
        this.robotToCam = robotToCam;
        this.poseEstimator = new PhotonPoseEstimator(fieldLayout, robotToCam);
        this.estConsumer = estConsumer;

        if (Robot.isSimulation()) {
            // Create the vision system simulation which handles cameras and targets on the field.
            visionSim = new VisionSystemSim("main");
            // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
            visionSim.addAprilTags(fieldLayout);
            // Create simulated camera properties. These can be set to mimic your actual camera.
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(15);
            cameraProp.setAvgLatencyMs(50);
            cameraProp.setLatencyStdDevMs(15);
            cameraSim = new PhotonCameraSim(camera, cameraProp);
            visionSim.addCamera(cameraSim, robotToCam);
        } else {
            visionSim = null;
            cameraSim = null;
        }
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
            pipelineResults.parallelStream().forEach(result -> {
                Optional<EstimatedRobotPose> estimatedPose;
                synchronized (poseEstimator) {
                    estimatedPose = poseEstimator.estimateCoprocMultiTagPose(result).or(() -> poseEstimator.estimateLowestAmbiguityPose(result));
                }
                updateEstimationStdDevs(estimatedPose, result.getTargets());
                if (RobotBase.isSimulation()) {
                    estimatedPose.ifPresentOrElse(est -> {
                        getSimDebugField()
                                .getObject("VisionEstimation")
                                .setPose(est.estimatedPose.toPose2d());
                    }, () -> {
                        getSimDebugField().getObject("VisionEstimation").setPoses();
                    });
                } // TODO: simulation

                estimatedPose.ifPresent(est -> {
                    synchronized (estConsumer) {
                        Matrix<N3, N1> estStdDevs = getEstimationStdDevs();
                        estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                    }
                });
            });
        });
    }

    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return visionSim.getDebugField();
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
