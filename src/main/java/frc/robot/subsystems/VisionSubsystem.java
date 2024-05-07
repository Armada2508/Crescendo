package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.lib.logging.NTLogger.log;

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
import frc.robot.Constants.Vision;
import frc.robot.Field;
import frc.robot.lib.logging.NTLogger;

public class VisionSubsystem extends SubsystemBase {

    private final PhotonCamera tagCam = new PhotonCamera(Vision.tagCameraName); 
    private final PhotonCamera noteCam = new PhotonCamera(Vision.noteCameraName); 
    private final PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(Vision.aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, tagCam, Vision.robotToCamera);
    private Optional<EstimatedRobotPose> latestEstimatedPose = Optional.empty();
    private PhotonPipelineResult tagResult = new PhotonPipelineResult();
    private PhotonPipelineResult noteResult = new PhotonPipelineResult();

    @Override
    public void periodic() {
        if (tagCam.isConnected()) {
            tagResult = tagCam.getLatestResult();
            latestEstimatedPose = photonPoseEstimator.update();
        }
        else {
            latestEstimatedPose = Optional.empty();
        }
        if (noteCam.isConnected()) {
            noteResult = noteCam.getLatestResult();
        }
        logVision();
    }

    public boolean canSeeTag() {
        if (!tagCam.isConnected()) return false;
        return tagResult.hasTargets();
    }

    public boolean canSeeNote() {
        if (!noteCam.isConnected()) return false;
        return noteResult.hasTargets();
    }

    /**
     * @return note yaw CCW+
     */
    public double getNoteYaw() {
        if (canSeeNote()) {
            return -noteResult.getBestTarget().getYaw();
        }
        return 0;
    }

    public double getNotePitch() {
        if (canSeeNote()) {
            return noteResult.getBestTarget().getPitch();
        }
        return 0;
    }

    public Optional<VisionResult> getVisionResult() {
        if (latestEstimatedPose.isEmpty()) return Optional.empty();
        Pose2d robotPose = latestEstimatedPose.get().estimatedPose.toPose2d();
        double timestampSeconds = latestEstimatedPose.get().timestampSeconds;
        if (!isValidPose(robotPose)) return Optional.empty();
        Matrix<N3, N1> stdDevs = getStdDevs(robotPose);
        NTLogger.log(this, "Last StdDevs", stdDevs);
        return Optional.of(new VisionResult(robotPose, timestampSeconds, stdDevs));
    }

    /**
     * Checks if a given pose is within the bounds of the FIELD.
     */
    private boolean isValidPose(Pose2d pose) {
        return (pose.getX() >= 0 && pose.getX() <= Field.fieldLength.in(Meters) && pose.getY() >= 0 && pose.getY() <= Field.fieldWidth.in(Meters));
    }
    
    private Matrix<N3, N1> getStdDevs(Pose2d estimatedPose) {
        int numTags = 0;
        double avgDistMeters = 0; 
        for (var target : tagResult.getTargets()) {
            var tagPose = photonPoseEstimator.getFieldTags().getTagPose(target.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDistMeters += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return Vision.singleTagVisionStdDevs;
        avgDistMeters /= numTags;
        NTLogger.log(this, "Average Distance to Tags (in.) RF", Meters.of(avgDistMeters).in(Inches)); // Logging

        // if (numTags > 1) return Vision.multiTagVisionStdDevs;

        if (avgDistMeters > Vision.maxAvgTagDistance.in(Meters)) return Vision.untrustedVisionStdDevs;
        // if (Vision.stageTags.contains(tagResult.getBestTarget().getFiducialId())) return Vision.multiTagVisionStdDevs; // Trust stage tags more for climbing
        return Vision.multiTagVisionStdDevs;
    }

    private void logVision() {
        if (canSeeTag()) {
            var target = tagResult.getBestTarget();
            int id = target.getFiducialId();
            double distance = Units.metersToInches(target.getBestCameraToTarget().getTranslation().getNorm());
            log(this, "Best Tag ID", id);
            log(this, "Best Tag Distance (in.) CF", distance); // Camera Frame
            log(this, "Num Tags Seen", tagResult.getTargets().size());
        }
        if (canSeeNote()) {
            var target = noteResult.getBestTarget();
            log(this, "Note Yaw", target.getYaw());
            log(this, "Note Pitch", target.getPitch());
        }
        latestEstimatedPose.ifPresent(p -> log(this, "Estimated Pose Z (in.)", Units.metersToInches(p.estimatedPose.getZ())));
        log(this, "Camera Connected Note", noteCam.isConnected());
        log(this, "Camera Connected Tag", tagCam.isConnected());
        log(this, "Can See Tag", canSeeTag());
        log(this, "Can See Note", canSeeNote());
    }

    /**
     * @see EstimatedRobotPose
     */
    public record VisionResult(
        Pose2d estimatedRobotPose,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {}

}
