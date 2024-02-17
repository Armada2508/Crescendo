package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Field;
import frc.robot.lib.Encoder;
import frc.robot.lib.drive.DriveCommand;
import frc.robot.lib.logging.Loggable;
import frc.robot.lib.logging.NTLogger;
import frc.robot.lib.motion.FollowTrajectory;
import frc.robot.lib.music.TalonMusic;
import frc.robot.lib.util.Util;

public class DriveSubsystem extends SubsystemBase implements Loggable {

    private final TalonFX talonL = new TalonFX(Drive.LID);
    private final TalonFX talonR = new TalonFX(Drive.RID);
    private final TalonFX talonLFollow = new TalonFX(Drive.LFollowID);
    private final TalonFX talonRFollow = new TalonFX(Drive.RFollowID);
    private final PigeonIMU pigeon = new PigeonIMU(Drive.pigeonID); 
    private final PIDController turnPIDController = new PIDController(Drive.turnPIDConfig.kP, Drive.turnPIDConfig.kI, Drive.turnPIDConfig.kD);
    private final VisionSubsystem vision; 
    private DifferentialDrivePoseEstimator poseEstimator;
    // private DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(Drive.diffKinematics, getAngle(), getLeftPosition(), getRightPosition(), new Pose2d());

    public DriveSubsystem(VisionSubsystem vision) {
        this.vision = vision;
        turnPIDController.setTolerance(Drive.turnDeadband.in(Degrees)); 
        configTalons();
        NTLogger.register(this);
        TalonMusic.addTalonFX(this, talonL, talonLFollow, talonR, talonRFollow);
    }

    @Override
    public void periodic() {
        if (pigeon.getState() != PigeonState.Ready) return;
        Optional<EstimatedRobotPose> visionPose = vision.getEstimatedGlobalPose();
        if (poseEstimator == null) { // Initialize field pose
            if (visionPose.isPresent()) {
                poseEstimator = new DifferentialDrivePoseEstimator(Drive.diffKinematics, getAngle(), getLeftPosition(), getRightPosition(), visionPose.get().estimatedPose.toPose2d());
            }
            return;
        }
        poseEstimator.update(getAngle(), getLeftPosition(), getRightPosition());
        if (visionPose.isPresent()) {
            Pose2d pose = visionPose.get().estimatedPose.toPose2d();
            if (vision.isValidPose(pose)) {
                poseEstimator.addVisionMeasurement(pose, visionPose.get().timestampSeconds, vision.getEstimationStdDevs(pose));
            }
        }   
        Field.simulatedField.setRobotPose(getFieldPose());
    }

    private void configTalons() {
        pigeon.configFactoryDefault();
        pigeon.setYaw(0);
        Util.factoryResetTalons(talonL, talonR, talonLFollow, talonRFollow);
        Util.brakeMode(talonL, talonR, talonLFollow, talonRFollow);
        talonLFollow.setControl(new StrictFollower(talonL.getDeviceID()));
        talonRFollow.setControl(new StrictFollower(talonR.getDeviceID()));
        talonR.setInverted(true);
        talonRFollow.setInverted(true);
        talonL.getConfigurator().apply(Drive.motionMagicConfig);
        talonR.getConfigurator().apply(Drive.motionMagicConfig);
        talonL.getConfigurator().apply(Drive.velocityLeftConfig);
        talonR.getConfigurator().apply(Drive.velocityRightConfig);
    }

    /**
     * 
     * @param velocity meters/second
     * @param acceleration meters/second^2
     */
    private void configMotionMagic(double velocity, double acceleration) {
        MotionMagicConfigs config = new MotionMagicConfigs();
        // Time period is 1s
        config.MotionMagicCruiseVelocity = toRotations(velocity);
        config.MotionMagicAcceleration = toRotations(acceleration);
        talonL.getConfigurator().apply(config);
        talonR.getConfigurator().apply(config);
    }

    /**
     * Drives the robot a certain distance relative to itself using motion magic
     * @param distance meters
     */
    private void driveDistance(Measure<Distance> distance) {
        MotionMagicVoltage request = new MotionMagicVoltage(0).withSlot(Drive.motionMagicSlot);
        double distanceRots = toRotations(distance.in(Meters));
        talonL.setControl(request.withPosition(talonL.getPosition().getValueAsDouble() + distanceRots));
        talonR.setControl(request.withPosition(talonR.getPosition().getValueAsDouble() + distanceRots));
    }

    /**
     * @param leftVelocity meters/second
     * @param rightVelocity meters/second
     */
    public void setVelocity(double leftVelocity, double rightVelocity) {
        VelocityVoltage request = new VelocityVoltage(0);
        talonL.setControl(request.withVelocity(toRotations(leftVelocity)));
        talonR.setControl(request.withVelocity(toRotations(rightVelocity)));
    }

    public void setVoltage(Measure<Voltage> left, Measure<Voltage> right) {
        VoltageOut request = new VoltageOut(0);
        talonL.setControl(request.withOutput(left.in(Volts)));
        talonR.setControl(request.withOutput(right.in(Volts)));
    }

    public void setSpeed(double leftSpeed, double rightSpeed) {
        talonL.setControl(new DutyCycleOut(leftSpeed));
        talonR.setControl(new DutyCycleOut(rightSpeed));
    }

    /**
     * 
     * @param distance meters relative to robot
     * @param velocity meters/second
     * @param acceleration meters/second^2
     * @return Command for the robot to drive distance using motion magic
     */
    public Command driveDistanceCommand(Measure<Distance> distance, double velocity, double acceleration) {
        final double deadbandRotations = toRotations(Drive.driveDeadband.in(Meters)); 
        return runOnce(() -> {
            configMotionMagic(velocity, acceleration);
            driveDistance(distance);
        })
        .andThen(Commands.waitUntil(() -> Util.epsilonEquals(talonL.getPosition().getValueAsDouble(), talonL.getClosedLoopReference().getValueAsDouble(), deadbandRotations)))
        .finallyDo(this::stop)
        .withName("Drive Distance Command");
    }

    /**
     * @param angle - angle in field frame 
     * @return Command for the robot to turn to angle in field frame
     */
    public Command turnCommand(Supplier<Measure<Angle>> angle) {
        turnPIDController.reset();
        turnPIDController.setSetpoint(0);
        return runEnd(() -> {
            double volts = turnPIDController.calculate(shortestPath(getFieldPose().getRotation().getDegrees(), angle.get().in(Degrees)));
            double voltsL = volts + Drive.velocityLeftConfig.kS * Math.signum(volts);
            double voltsR = volts + Drive.velocityRightConfig.kS * Math.signum(volts);
            voltsL = MathUtil.clamp(voltsL, -Drive.maxTurnPIDVoltage.in(Volts), Drive.maxTurnPIDVoltage.in(Volts));
            voltsR = MathUtil.clamp(voltsR, -Drive.maxTurnPIDVoltage.in(Volts), Drive.maxTurnPIDVoltage.in(Volts));
            setVoltage(Volts.of(voltsL), Volts.of(-voltsR));
        }, this::stop)
        .until(turnPIDController::atSetpoint)
        .withName("Turn Command");
    }

    /**
     * @param angle - angle in field frame 
     * @return Command for the robot to turn to angle in field frame
     */
    public Command turnCommand(Measure<Angle> angle) {
        return turnCommand(() -> angle);
    }

    /**
     * Returns the shorter error considering a wrap around bounded [-180, 180] degrees, CCW positive
     */
    private double shortestPath(double current, double target) {
        double error1 = Constants.degreesPerRotation + target - current;
        double error2 = target - current;
        if (Math.abs(error1) < Math.abs(error2)) return error1;
        return error2;
    }

    public Command trajectoryToPoseCommand(Supplier<Pose2d> targetPose) {
        return runOnce(() -> {
            System.out.println("Creating trajectory");
            Trajectory trajectory = TrajectoryGenerator.generateTrajectory(getFieldPose(), new ArrayList<>(), targetPose.get(), Drive.trajectoryConfig);
            System.out.println("Done creating trajectory");
            Field.simulatedField.getObject("traj").setTrajectory(trajectory);
            FollowTrajectory.getCommandTalon(trajectory, Field.origin, this::getFieldPose, this::setVelocity, this).schedule();
        }).withName("Generating Trajectory Command");
    }

    public Command trajectoryToPoseCommand(Pose2d targetPose) {
        return trajectoryToPoseCommand(() -> targetPose);
    }

    public void stop() {
        talonL.setControl(new NeutralOut());
        talonR.setControl(new NeutralOut());
    }

    /**
     * @param distance meters
     */
    private double toRotations(double distance) {
        return Encoder.fromDistance(distance, Drive.gearRatio, Drive.wheelDiameter.in(Meters));
    }

    private double getLeftPosition() {
        return Encoder.toDistance(talonL.getPosition().getValueAsDouble(), Drive.gearRatio, Drive.wheelDiameter.in(Meters));
    }

    private double getRightPosition() {
        return Encoder.toDistance(talonR.getPosition().getValueAsDouble(), Drive.gearRatio, Drive.wheelDiameter.in(Meters));
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(-pigeon.getYaw()); 
    }

    public Pose2d getFieldPose() {
        return (poseEstimator == null) ? Field.origin : poseEstimator.getEstimatedPosition();
    }

    public void resetFieldPose() {
        if (poseEstimator != null) {
            poseEstimator.resetPosition(getAngle(), getLeftPosition(), getRightPosition(), new Pose2d());
        }
    }

    public Command joystickDriveCommand(DoubleSupplier joystickSpeed, DoubleSupplier joystickTurn, DoubleSupplier joystickTrim, BooleanSupplier joystickSlow) {
        return new DriveCommand(joystickSpeed, joystickTurn, joystickTrim, joystickSlow, Drive.joystickDriveConfig, this::setSpeed, this::stop, this);
    }

    @Override
    public Map<String, Object> log(Map<String, Object> map) {
        if (pigeon.getState() == PigeonState.Ready) {
            map.put("Pigeon Yaw", pigeon.getYaw());
        }
        map.put("Robot Angle", getFieldPose().getRotation().getDegrees());
        NTLogger.putTalonLog(talonL, "Left TalonFX", map);
        NTLogger.putTalonLog(talonLFollow, "Left Follow TalonFX", map);
        NTLogger.putTalonLog(talonR, "Right TalonFX", map);
        NTLogger.putTalonLog(talonRFollow, "Right Follow TalonFX", map);
        NTLogger.putSubsystemLog(this, map);
        return map;
    }
}
