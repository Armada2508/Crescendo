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

    private final PhotonCamera tagCam = new PhotonCamera(Vision.tagCameraName); 
    private final PhotonCamera noteCam = new PhotonCamera(Vision.noteCameraName); 
    private final PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(Vision.aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, tagCam, Vision.robotToCamera);
    private PhotonPipelineResult tagResult;
    private PhotonPipelineResult noteResult;

    public VisionSubsystem() {
        NTLogger.register(this);
    }

    @Override
    public void periodic() {
        if (tagCam.isConnected()) {
            tagResult = tagCam.getLatestResult();
        }
        if (noteCam.isConnected()) {
            noteResult = noteCam.getLatestResult();
        }
    }

    public boolean canSeeTag() {
        if (!tagCam.isConnected()) return false;
        return tagResult.hasTargets();
    }

    public boolean canSeeNote() {
        if (!noteCam.isConnected()) return false;
        return noteResult.hasTargets();
    }

    public double getNoteYaw() {
        if (canSeeNote()) {
            return noteResult.getBestTarget().getYaw();
        }
        return 0;
    }

    public double getNotePitch() {
        if (canSeeNote()) {
            return noteResult.getBestTarget().getPitch();
        }
        return 0;
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        if (!tagCam.isConnected()) return Optional.empty();
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
        if (canSeeTag()) {
            var target = tagResult.getBestTarget();
            int id = target.getFiducialId();
            double distance = Units.metersToInches(target.getBestCameraToTarget().getTranslation().getNorm());
            map.put("Best Tag ID", id);
            map.put("Best Tag Distance", distance);
        }
        if (canSeeNote()) {
            var target = noteResult.getBestTarget();
            map.put("Note Yaw", target.getYaw());
            map.put("Note Pitch", target.getPitch());
        }
        map.put("Camera Connected Note", tagCam.isConnected());
        map.put("Camera Connected Tag", tagCam.isConnected());
        map.put("Can See Tag", canSeeTag());
        map.put("Can See Note", canSeeNote());
        return map;
    }

}
