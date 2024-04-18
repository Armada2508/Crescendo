package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class Field {
    
    // Origin is always the blue alliance origin
    public static final Pose2d origin = new Pose2d();
    public static final Field2d simulatedField = new Field2d();
    public static final Measure<Distance> fieldLength = Inches.of(651.25);
    public static final Measure<Distance> fieldWidth = Inches.of(323.25); // Also kind of like the height of the field
    public static final Measure<Angle> minAngle = Degrees.of(-180);
    public static final Measure<Angle> maxAngle = Degrees.of(180);
    public static final Measure<Distance> subWooferLength = Inches.of(36.125);
    public static final Translation2d blueSpeakerPosition = new Translation2d(Meters.of(0), Meters.of(5.55));
    public static final Translation2d redSpeakerPosition = new Translation2d(Field.fieldLength, Meters.of(5.55));    
    // Position of the front of the SUBWOOFER, facing the SPEAKER
    public static final Pose2d blueSpeakerBaseScorePos = new Pose2d(subWooferLength.plus(Constants.halfRobotLength).plus(Inches.of(4)), Meters.of(5.55), Rotation2d.fromDegrees(0));    
    public static final Pose2d redSpeakerBaseScorePos = new Pose2d(Field.fieldLength.minus(subWooferLength).minus(Constants.halfRobotLength).minus(Inches.of(4)), Meters.of(5.55), Rotation2d.fromDegrees(180));  
    // Position in front of the AMP, facing the AMP  
    public static final Pose2d blueAmpScorePos = new Pose2d(Inches.of(72.5), fieldWidth.minus(Constants.halfRobotLength), Rotation2d.fromDegrees(90));
    public static final Pose2d redAmpScorePos = new Pose2d(fieldLength.minus(Inches.of(72.5)), fieldWidth.minus(Constants.halfRobotLength), Rotation2d.fromDegrees(90));
    // Blue Notes, plus fudge for AUTO
    public static final Translation2d blueTopNotePos = new Translation2d(Inches.of(114), Inches.of(275.625)); 
    public static final Translation2d blueMidNotePos = new Translation2d(Inches.of(114), Inches.of(218.625)); 
    public static final Translation2d blueLowNotePos = new Translation2d(Inches.of(114), Inches.of(161.625)); 
    // Red Notes, plus fudge for AUTO
    public static final Translation2d redTopNotePos = new Translation2d(Field.fieldLength.minus(Inches.of(114)), Inches.of(275.625)); 
    public static final Translation2d redMidNotePos = new Translation2d(Field.fieldLength.minus(Inches.of(114)), Inches.of(218.625)); 
    public static final Translation2d redLowNotePos = new Translation2d(Field.fieldLength.minus(Inches.of(114)), Inches.of(161.625)); 
    
    
    // Blue Stage April Tag Poses
    public static final Pose2d blueStageCenter = new Pose2d(Meters.of(5.32), Meters.of(4.11), Rotation2d.fromDegrees(0));
    public static final Pose2d blueStageLeft = new Pose2d(Meters.of(4.64), Meters.of(4.5), Rotation2d.fromDegrees(120));
    public static final Pose2d blueStageRight = new Pose2d(Meters.of(4.64), Meters.of(3.71), Rotation2d.fromDegrees(-120));
    public static final Measure<Distance> chainToStage = Inches.of(16.625);
    public static final Measure<Distance> chainClimbOffset = chainToStage.plus(Inches.of(-6));

    public static final Pose2d blueChainCenter = new Pose2d(Meters.of(5.32).plus(chainClimbOffset), Meters.of(4.11), blueStageCenter.getRotation());
    public static final Translation2d leftChainVector = new Translation2d(chainClimbOffset, Meters.of(0)).rotateBy(blueStageLeft.getRotation());
    public static final Pose2d blueChainLeft = new Pose2d(blueStageLeft.getTranslation().plus(leftChainVector), blueStageLeft.getRotation());
    public static final Translation2d rightChainVector = new Translation2d(chainClimbOffset, Meters.of(0)).rotateBy(blueStageRight.getRotation());
    public static final Pose2d blueChainRight = new Pose2d(blueStageRight.getTranslation().plus(rightChainVector), blueStageRight.getRotation());
    public static final Pose2d redChainCenter = new Pose2d(Field.fieldLength.in(Meters) - blueChainCenter.getX(), blueChainCenter.getY(), Rotation2d.fromDegrees(180));
    public static final Pose2d redChainLeft = new Pose2d(Field.fieldLength.in(Meters) - blueChainRight.getX(), blueChainRight.getY(), Rotation2d.fromDegrees(-60));
    public static final Pose2d redChainRight = new Pose2d(Field.fieldLength.in(Meters) - blueChainLeft.getX(), blueChainLeft.getY(), Rotation2d.fromDegrees(60));

    public enum Note {
        TOP,
        MID,
        LOW
    }

    public static Translation2d getNote(Note note) {
        return switch (note) {
            case TOP -> Robot.onRedAlliance() ? Field.redTopNotePos : Field.blueTopNotePos;
            case MID -> Robot.onRedAlliance() ? Field.redMidNotePos : Field.blueMidNotePos;
            case LOW -> Robot.onRedAlliance() ? Field.redLowNotePos : Field.blueLowNotePos;
            default -> throw new IllegalArgumentException("Illegal Note: " + note);
        };
    }

    public static Pose2d getNearestChain(Pose2d pose) {
        Pose2d chainCenter = blueChainCenter;
        Pose2d chainLeft = blueChainLeft;
        Pose2d chainRight = blueChainRight;
        if (Robot.onRedAlliance()) {
            chainCenter = redChainCenter;
            chainLeft = redChainLeft;
            chainRight = redChainRight;
        }
        double distanceCenter = pose.getTranslation().getDistance(chainCenter.getTranslation());
        double distanceLeft = pose.getTranslation().getDistance(chainLeft.getTranslation());
        double distanceRight = pose.getTranslation().getDistance(chainRight.getTranslation());
        double distance = Math.min(Math.min(distanceLeft, distanceRight), distanceCenter);
        if (distance == distanceCenter) {
            return chainCenter;
        } 
        else if (distance == distanceLeft) {
            return chainLeft;
        }
        else {
            return chainRight;
        }
    }

    public static Measure<Distance> getDistanceToSpeaker(Pose2d pose) {
        Translation2d speakerPos = (Robot.onRedAlliance()) ? Field.redSpeakerPosition : Field.blueSpeakerPosition;
        return Meters.of(pose.getTranslation().getDistance(speakerPos));
    }

}
