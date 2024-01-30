package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import java.util.Map;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Field;
import frc.robot.Constants.Vision;
import frc.robot.lib.logging.Loggable;
import frc.robot.lib.logging.NTLogger;

public class VisionSubsystem extends SubsystemBase implements Loggable {

    private final PhotonCamera cam = new PhotonCamera(Vision.cameraName); 
    private final PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(Vision.aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam, Vision.robotToCamera);
    private PhotonPipelineResult result;

    public VisionSubsystem() {
        NTLogger.register(this);
    }

    @Override
    public void periodic() {
        if (!cam.isConnected()) return;
        result = cam.getLatestResult();
    }

    public boolean canSeeTag() {
        if (!cam.isConnected()) return false;
        return result.hasTargets();
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        if (!cam.isConnected()) return Optional.empty();
        return photonPoseEstimator.update();
    }

    public boolean isValidPose(Pose2d pose) {
        return (pose.getX() >= 0 && pose.getX() <= Field.fieldLength.in(Meters) && pose.getY() >= 0 && pose.getY() <= Field.fieldWidth.in(Meters));
    }
    
    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) { //! Should be actually implemented somehow
        return Vision.poseStdDevs;
    }

    @Override
    public Map<String, Object> log(Map<String, Object> map) {
        double distance = 0;
        if (canSeeTag()) {
            var target = result.getBestTarget();
            distance = Units.metersToInches(target.getBestCameraToTarget().getTranslation().getNorm());
        }
        map.put("Can See Tag", canSeeTag());
        map.put("Distance", distance);
        return map;
    }

}
