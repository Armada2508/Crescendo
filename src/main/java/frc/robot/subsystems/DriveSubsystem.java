package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.lib.logging.NTLogger.log;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Shooter;
import frc.robot.Field;
import frc.robot.Robot;
import frc.robot.commands.DriveCommands;
import frc.robot.lib.Encoder;
import frc.robot.lib.logging.NTLogger;
import frc.robot.lib.motion.FollowTrajectory;
import frc.robot.lib.music.TalonMusic;
import frc.robot.lib.util.Util;
import frc.robot.subsystems.VisionSubsystem.VisionResult;

public class DriveSubsystem extends SubsystemBase {

    private final TalonFX talonL = new TalonFX(Drive.LID);
    private final TalonFX talonR = new TalonFX(Drive.RID);
    private final TalonFX talonLFollow = new TalonFX(Drive.LFollowerID);
    private final TalonFX talonRFollow = new TalonFX(Drive.RFollowerID);
    private final PigeonIMU pigeon = new PigeonIMU(Drive.pigeonID); 
    private final PIDController turnPIDController = new PIDController(Drive.turnPIDConfig.kP, Drive.turnPIDConfig.kI, Drive.turnPIDConfig.kD);
    private final Supplier<Optional<VisionResult>> visionSupplier; 
    private DifferentialDrivePoseEstimator poseEstimator;

    public DriveSubsystem(Supplier<Optional<VisionResult>> visionSupplier) {
        this.visionSupplier = visionSupplier;
        turnPIDController.setTolerance(Drive.turnDeadband.in(Degrees)); 
        turnPIDController.enableContinuousInput(Field.minAngle.in(Degrees), Field.maxAngle.in(Degrees));
        configTalons();
        TalonMusic.addTalonFX(this, talonL, talonR);
    }

    @Override
    public void periodic() {
        boolean pigeonValid = (pigeon.getState() == PigeonState.Ready);
        Optional<VisionResult> result = visionSupplier.get();
        if (poseEstimator == null && pigeonValid) { // Initialize field pose
            result.ifPresent((r) -> {
                poseEstimator = new DifferentialDrivePoseEstimator(Drive.diffKinematics, getAngle(), getLeftPosition(), getRightPosition(), r.estimatedRobotPose());
            });
        }
        else if (pigeonValid) { // Pose is initalized and ready to be updated
            poseEstimator.update(getAngle(), getLeftPosition(), getRightPosition());
            result.ifPresent((r) -> {
                poseEstimator.addVisionMeasurement(r.estimatedRobotPose(), r.timestampSeconds(), r.visionMeasurementStdDevs());
            });
        }
        logDrive();
    }

    private void configTalons() {
        pigeon.configFactoryDefault();
        pigeon.setYaw(0);
        talonL.setPosition(0);
        talonR.setPosition(0);
        Util.factoryResetTalons(talonL, talonR, talonLFollow, talonRFollow);
        Util.brakeMode(talonL, talonR, talonLFollow, talonRFollow);
        talonLFollow.setControl(new StrictFollower(talonL.getDeviceID()));
        talonRFollow.setControl(new StrictFollower(talonR.getDeviceID()));
        talonR.setInverted(true);
        talonRFollow.setInverted(true);
        talonL.getConfigurator().apply(Drive.velocityLeftConfig);
        talonR.getConfigurator().apply(Drive.velocityRightConfig);
    }

    /**
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

    public void setSpeed(double leftSpeed, double rightSpeed) {
        talonL.setControl(new DutyCycleOut(leftSpeed));
        talonR.setControl(new DutyCycleOut(rightSpeed));
    }

    private void setVoltage(Measure<Voltage> left, Measure<Voltage> right) {
        VoltageOut request = new VoltageOut(0);
        talonL.setControl(request.withOutput(left.in(Volts)));
        talonR.setControl(request.withOutput(right.in(Volts)));
    }

    /**
     * @param leftVelocity meters/second
     * @param rightVelocity meters/second
     */
    private void setVelocity(double leftVelocity, double rightVelocity) {
        VelocityVoltage request = new VelocityVoltage(0);
        talonL.setControl(request.withVelocity(toRotations(leftVelocity)));
        talonR.setControl(request.withVelocity(toRotations(rightVelocity)));
    }

    public Command setVoltageCommand(Measure<Voltage> leftVoltage, Measure<Voltage> rightVoltage) {
        return runOnce(() -> setVoltage(leftVoltage, rightVoltage));
    }

    public Command setVelocityCommand(Measure<Velocity<Distance>> leftVelocity, Measure<Velocity<Distance>> rightVelocity) {
        return runOnce(() -> {
            setVelocity(leftVelocity.in(MetersPerSecond), rightVelocity.in(MetersPerSecond));
        });
    }

    public Command motionMagicVelocityCommand(Measure<Velocity<Distance>> leftVelocity, 
            Measure<Velocity<Distance>> rightVelocity, Measure<Velocity<Velocity<Distance>>> acceleration) {
        return runOnce(() -> {
            configMotionMagic(0, Math.abs(acceleration.in(MetersPerSecondPerSecond)));
            MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0);
            talonL.setControl(request.withVelocity(toRotations(leftVelocity.in(MetersPerSecond))));
            talonR.setControl(request.withVelocity(toRotations(rightVelocity.in(MetersPerSecond))));
        });
    }

    /**
     * @param distance distance to travel
     * @param velocity meters/second, negative means go backwards
     * @return Command for the robot to drive a distance using velocity control
     */
    public Command driveDistanceVelCommand(Measure<Distance> distance, Measure<Velocity<Distance>> velocity) {
        return setVelocityCommand(velocity, velocity)
        .andThen(Commands.waitSeconds(Math.abs(distance.in(Meters) / velocity.in(MetersPerSecond))))
        .finallyDo(this::stop)
        .withName("Drive Distance Velocity");
    }

    private Measure<Angle> targetAngle = getFieldAngle();
    /**
     * Has inbuilt safety to an uninitalized field pose
     * @param angle - angle in field frame 
     * @return Command for the robot to turn to angle in field frame
     */
    public Command turnCommand(Supplier<Measure<Angle>> angle) {
        return runOnce(() -> { // Capture state
            targetAngle = angle.get();
            turnPIDController.reset();
            turnPIDController.setSetpoint(targetAngle.in(Degrees));
        }) 
        .andThen(runEnd(() -> {
            if (!hasInitalizedFieldPose()) {
                turnPIDController.calculate(turnPIDController.getSetpoint());
                return;
            }
            double volts = turnPIDController.calculate(getFieldAngle().in(Degrees));
            double voltsL = volts + Drive.minTurnPIDVoltage.in(Volts) * Math.signum(volts);
            double voltsR = volts + Drive.minTurnPIDVoltage.in(Volts) * Math.signum(volts);
            voltsL = MathUtil.clamp(voltsL, -Drive.maxTurnPIDVoltage.in(Volts), Drive.maxTurnPIDVoltage.in(Volts));
            voltsR = MathUtil.clamp(voltsR, -Drive.maxTurnPIDVoltage.in(Volts), Drive.maxTurnPIDVoltage.in(Volts));
            setVoltage(Volts.of(-voltsL), Volts.of(voltsR));
        }, this::stop))
        .until(turnPIDController::atSetpoint)
        .withName("Turn");
    }

    /**
     * Clamped Cubic Spline 
     */
    public Trajectory generateTrajectory(Pose2d targetPose, TrajectoryConfig config) {
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(getFieldPose(), new ArrayList<>(), targetPose, config);
        NTLogger.log(this, "Cubic Trajectory", trajectory);
        return trajectory;
    }

    /**
     * Quintic Hermite Spline 
     * @param waypoints list of waypoints including end pose, doesn't include start pose
     */
    public Trajectory generateTrajectory(List<Pose2d> waypoints, TrajectoryConfig config) {
        waypoints.add(0, getFieldPose());
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints, config);
        NTLogger.log(this, "Quintic Trajectory", trajectory);
        return trajectory;
    }

    public Command trajectoryToPoseCommand(Supplier<Trajectory> trajectory) {
        return Commands.defer(() -> {
            if (!hasInitalizedFieldPose()) {
                return Commands.none();
            }
            return FollowTrajectory.ramseteControllerCommand(trajectory.get(), Field.origin, this::getFieldPose, this::setVelocity, this);
        }, Set.of(this));
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

    public Measure<Angle> getFieldAngle() {
        return Degrees.of(getFieldPose().getRotation().getDegrees());
    }

    public boolean hasInitalizedFieldPose() {
        return poseEstimator != null;
    }

    public Command joystickDriveCommand(DoubleSupplier joystickSpeed, DoubleSupplier joystickTurn, DoubleSupplier joystickTrim, 
            BooleanSupplier noteModeEnabled, DoubleSupplier noteAngle) {
        return DriveCommands.drive(joystickSpeed, joystickTurn, joystickTrim, noteModeEnabled, noteAngle, this);
    }

    private void logDrive() {
        if (pigeon.getState() == PigeonState.Ready) {
            log(this, "Pigeon Yaw", pigeon.getYaw());
        }
        log(this, "Robot Pose", getFieldPose());
        log(this, "Robot Angle", getFieldAngle().in(Degrees));
        log(this, "Target Angle", targetAngle.in(Degrees));
        log(this, "Turn PID Position Error", turnPIDController.getPositionError());
        log(this, "Turn PID Velocity Error", turnPIDController.getVelocityError());
        log(this, "Distance to Speaker", Field.getDistanceToSpeaker(getFieldPose()).in(Inches));
        Translation2d speaker = Field.blueSpeakerPosition;
        if (Robot.onRedAlliance()) speaker = Field.redSpeakerPosition;
        log(this, "Lateral Speaker Distance", getFieldPose().getY() - speaker.getY());
        log(this, "In Range", Field.getDistanceToSpeaker(getFieldPose()).lte(Shooter.maxShootDistance));
        log(this, "Has Initalized Pose", hasInitalizedFieldPose());
        log(this, "Left TalonFX", talonL);
        log(this, "Left Follow TalonFX", talonLFollow);
        log(this, "Right TalonFX", talonR);
        log(this, "Right Follow TalonFX", talonRFollow);
        log(this, "Subsystem", this);
    }
    
}
