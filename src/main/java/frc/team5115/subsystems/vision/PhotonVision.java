package frc.team5115.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Constants.VisionConstants;
import frc.team5115.subsystems.drive.Drivetrain;
import java.io.IOException;
import java.nio.file.Path;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class PhotonVision extends SubsystemBase {
    private final Drivetrain drivetrain;
    private final PhotonCamera camera;
    private final AprilTagFieldLayout fieldLayout;
    private final PhotonPoseEstimator poseEstimator;

    public PhotonVision(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        camera = new PhotonCamera(VisionConstants.cameraName);
        try {
            Path fieldLayoutPath =
                    Filesystem.getDeployDirectory().toPath().resolve("Bunnybots_2024.json");
            fieldLayout = new AprilTagFieldLayout(fieldLayoutPath);
        } catch (IOException e) {
            e.printStackTrace();
            throw new RuntimeException("Failed to load field layout file");
        }

        poseEstimator =
                new PhotonPoseEstimator(
                        fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, camera, VisionConstants.robotToCam);
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        poseEstimator.setReferencePose(prevEstimatedRobotPose);
        var pose = poseEstimator.update();
        return pose;
    }

    @Override
    public void periodic() {
        var option = getEstimatedGlobalPose(drivetrain.getPose());
        Logger.recordOutput("Vision/HasMeasurement", option.isPresent());
        if (option.isPresent()) {
            EstimatedRobotPose pose = option.get();
            drivetrain.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
            Logger.recordOutput("Vision/EstimatedPose", pose.estimatedPose);
        }
    }
}
