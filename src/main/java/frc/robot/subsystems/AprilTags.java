package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class AprilTags implements Subsystem {
    private final PhotonCamera m_camera;
    private final PhotonPoseEstimator m_photonPoseEstimator;
    private final StructPublisher<Pose3d> posePublisher;

    private final Swerve m_drivetrain;

    public AprilTags(String cameraName, Transform3d robotToCamera, Swerve drivetrain) {
        var fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        posePublisher = NetworkTableInstance.getDefault().getTable("Pose Estimates").getStructTopic(cameraName, Pose3d.struct).publish();

        m_camera = new PhotonCamera(cameraName);
        m_photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToCamera);
        m_drivetrain = drivetrain;

        register();
    }

    @Override
    public void periodic() {
        var results = m_camera.getAllUnreadResults();

        for (var result : results) {
            var estimate = m_photonPoseEstimator.update(result);

            if (estimate.isPresent()) {
                var est = estimate.get();
                posePublisher.set(est.estimatedPose);
                m_drivetrain.addVisionMeasurement(est.estimatedPose.toPose2d(),
                        Utils.fpgaToCurrentTime(est.timestampSeconds));
            }
        }
    }
}
