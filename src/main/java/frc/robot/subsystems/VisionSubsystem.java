package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Map;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Field;
import frc.robot.Constants.Vision;
import frc.robot.lib.logging.Loggable;
import frc.robot.lib.logging.NTLogger;

public class VisionSubsystem extends SubsystemBase implements Loggable {

    private final PhotonCamera tagCam = new PhotonCamera(Vision.tagCameraName); 
    private final PhotonCamera noteCam = new PhotonCamera(Vision.noteCameraName); 
    private final PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(Vision.aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, tagCam, Vision.robotToCamera);
    private Optional<EstimatedRobotPose> latestEstimatedPose = Optional.empty();
    private PhotonPipelineResult tagResult = new PhotonPipelineResult();
    private PhotonPipelineResult noteResult = new PhotonPipelineResult();
    private Measure<Distance> lastAvgDist = Inches.of(-1);
    private Matrix<N3, N1> lastStdDevs = VecBuilder.fill(0, 0, 0);

    public VisionSubsystem() {
        NTLogger.register(this);
    }

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

    public double getNoteDistance() {
        double camToNoteDistance = PhotonUtils.calculateDistanceToTargetMeters(Vision.robotToCamera.getY(), 0, Math.toRadians(0), Math.toRadians(getNotePitch())); //! verify to make sure this works
        double camHeight = Vision.robotToCamera.getY();
        return Math.abs(Math.sqrt(Math.pow(camToNoteDistance, 2) - Math.pow(camHeight, 2)));
    }

    public Optional<VisionResult> getVisionResult() {
        if (latestEstimatedPose.isEmpty()) return Optional.empty();
        Pose2d robotPose = latestEstimatedPose.get().estimatedPose.toPose2d();
        double timestampSeconds = latestEstimatedPose.get().timestampSeconds;
        if (!isValidPose(robotPose)) return Optional.empty();
        lastStdDevs = getStdDevs(robotPose);
        return Optional.of(new VisionResult(robotPose, timestampSeconds, lastStdDevs));
    }

    /**
     * Checks if a given pose is within the bounds of the FIELD.
     */
    private boolean isValidPose(Pose2d pose) {
        return (pose.getX() >= 0 && pose.getX() <= Field.fieldLength.in(Meters) && pose.getY() >= 0 && pose.getY() <= Field.fieldWidth.in(Meters));
    }
    
    private Matrix<N3, N1> getStdDevs(Pose2d estimatedPose) {
        //? Still might wanna scale stdevs by avg distance 
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
        lastAvgDist = Meters.of(avgDistMeters); // Logging

        if (numTags > 1) return Vision.multiTagVisionStdDevs;

        if (avgDistMeters > Vision.maxAvgTagDistance.in(Meters)) return Vision.untrustedVisionStdDevs;
        if (Vision.stageTags.contains(tagResult.getBestTarget().getFiducialId())) return Vision.multiTagVisionStdDevs; // Trust stage tags more for climbing
        return Vision.singleTagVisionStdDevs;
    }

    // public Pose2d getNotePose() {
    //     if (!canSeeNote()) return null;
    //     Translation2d noteTranslation = new Translation2d(getNoteDistance(), getNoteYaw());
    //     Rotation2d noteRotations = new Rotation2d(Degrees.of(getNoteYaw()));
    //     return new Pose2d(noteTranslation, noteRotations);
    // }

    @Override
    public Map<String, Object> log(Map<String, Object> map) {
        if (canSeeTag()) {
            var target = tagResult.getBestTarget();
            int id = target.getFiducialId();
            double distance = Units.metersToInches(target.getBestCameraToTarget().getTranslation().getNorm());
            map.put("Best Tag ID", id);
            map.put("Best Tag Distance (in.) CF", distance); // Camera Frame
            map.put("Num Tags Seen", tagResult.getTargets().size());
        }
        if (canSeeNote()) {
            var target = noteResult.getBestTarget();
            map.put("Note Yaw", target.getYaw());
            map.put("Note Pitch", target.getPitch());
        }
        latestEstimatedPose.ifPresent(p -> map.put("Estimated Pose Z (in.)", Units.metersToInches(p.estimatedPose.getZ())));
        map.put("Camera Connected Note", noteCam.isConnected());
        map.put("Camera Connected Tag", tagCam.isConnected());
        map.put("Can See Tag", canSeeTag());
        map.put("Can See Note", canSeeNote());
        map.put("Average Distance to Tags (in.) RF", lastAvgDist.in(Inches)); // Robot Frame
        map.put("Last Stdevs", lastStdDevs);
        return map;
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
