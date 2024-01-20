package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision;

public class VisionSubsystem extends SubsystemBase {

    private final PhotonCamera cam = new PhotonCamera(Vision.cameraName); 
    private final PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(Vision.aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam, Vision.robotToCamera);
    private PhotonPipelineResult result;

    @Override
    public void periodic() {
        result = cam.getLatestResult();
        if (result.hasTargets()) {
            var target = result.getBestTarget();
            System.out.println("Distance: " + Units.metersToInches(target.getBestCameraToTarget().getTranslation().getNorm()));
        }
    }

    public boolean canSeeTag() {
        return result.hasTargets();
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        return photonPoseEstimator.update();
    }

}
