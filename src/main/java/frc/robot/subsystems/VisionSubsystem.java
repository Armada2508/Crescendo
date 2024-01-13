package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision;
import frc.robot.lib.logging.LogUtil;

public class VisionSubsystem extends SubsystemBase {

    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private final PhotonCamera camera = new PhotonCamera(Vision.cameraName); 

    @Override
    public void periodic() {
        PhotonPipelineResult result = camera.getLatestResult();
        boolean hasTargets = result.hasTargets();
        if (hasTargets) {
            PhotonTrackedTarget target = result.getBestTarget();
            Transform3d cameraToTag = target.getBestCameraToTarget();
            int tagID = target.getFiducialId();
            LogUtil.printFormatted("ID Pose", tagID, cameraToTag);
            if (aprilTagFieldLayout.getTagPose(tagID).isPresent()) {
            //     Pose3d fieldRelativeTagPose = aprilTagFieldLayout.getTagPose(tagID).get();
            //     Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(cameraToTag, fieldRelativeTagPose, Vision.cameraToRobot);
            //     System.out.println(robotPose);
            }
        }
    }

}
