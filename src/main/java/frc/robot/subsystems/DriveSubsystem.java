package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

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
import frc.robot.lib.util.Util;

public class DriveSubsystem extends SubsystemBase implements Loggable {

    private final TalonFX talonL = new TalonFX(Drive.LID);
    private final TalonFX talonR = new TalonFX(Drive.RID);
    private final TalonFX talonLFollow = new TalonFX(Drive.LFollowID);
    private final TalonFX talonRFollow = new TalonFX(Drive.RFollowID);
    private final PigeonIMU pigeon = new PigeonIMU(Drive.pigeonID); 
    private final PIDController turnPIDController = new PIDController(Drive.turnPIDConfig.kP, Drive.turnPIDConfig.kI, Drive.turnPIDConfig.kD);
    private final VisionSubsystem vision; 
    // private DifferentialDrivePoseEstimator poseEstimator;
    private DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(Drive.diffKinematics, getAngle(), getLeftPosition(), getRightPosition(), new Pose2d());

    public DriveSubsystem(VisionSubsystem vision) {
        this.vision = vision;
        turnPIDController.setTolerance(Drive.turnDeadband.in(Degrees)); 
        configTalons();
        NTLogger.register(this);
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
        Util.factoryResetTalons(talonL, talonR, talonLFollow, talonRFollow);
        Util.brakeMode(talonL, talonR, talonLFollow, talonRFollow);
        talonLFollow.setControl(new StrictFollower(talonL.getDeviceID()));
        talonRFollow.setControl(new StrictFollower(talonR.getDeviceID()));
        talonR.setInverted(true);
        talonRFollow.setInverted(true);
        talonL.getConfigurator().apply(Drive.motionMagicConfig);
        talonR.getConfigurator().apply(Drive.motionMagicConfig);
        talonL.getConfigurator().apply(Drive.velocityConfig);
        talonR.getConfigurator().apply(Drive.velocityConfig);
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
        MotionMagicVoltage request = new MotionMagicVoltage(0);
        double distanceRots = toRotations(distance.in(Meters));
        talonL.setControl(request.withPosition(talonL.getPosition().getValueAsDouble() + distanceRots));
        talonR.setControl(request.withPosition(talonR.getPosition().getValueAsDouble() + distanceRots));
    }

    /**
     * @param leftVelocity meters/second
     * @param rightVelocity meters/second
     */
    private void setVelocity(double leftVelocity, double rightVelocity) {
        VelocityVoltage request = new VelocityVoltage(0).withSlot(Drive.velocitySlot);
        talonL.setControl(request.withVelocity(toRotations(leftVelocity)));
        talonR.setControl(request.withVelocity(toRotations(rightVelocity)));
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
            double speed = turnPIDController.calculate(shortestPath(getFieldPose().getRotation().getDegrees(), angle.get().in(Degrees)));
            speed = MathUtil.clamp(speed, -Drive.maxTurnPIDSpeed, Drive.maxTurnPIDSpeed);
            setSpeed(-speed, speed);
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
        return Rotation2d.fromDegrees(pigeon.getYaw()); //? I think this is right
    }

    public Pose2d getFieldPose() {
        return (poseEstimator == null) ? Field.origin : poseEstimator.getEstimatedPosition();
    }

    public Command joystickDriveCommand(DoubleSupplier joystickSpeed, DoubleSupplier joystickTurn, DoubleSupplier joystickTrim, BooleanSupplier joystickSlow) {
        return new DriveCommand(joystickSpeed, joystickTurn, joystickTrim, joystickSlow, Drive.joystickDriveConfig, this::setSpeed, this::stop, this);
    }

    @Override
    public Map<String, Object> log(Map<String, Object> map) {
        map.put("Pigeon Yaw", pigeon.getYaw());
        return Util.mergeMaps(map, NTLogger.getTalonLog(talonL), NTLogger.getTalonLog(talonR), 
            NTLogger.getTalonLog(talonLFollow), NTLogger.getTalonLog(talonRFollow), NTLogger.getSubsystemLog(this));
    }
}
