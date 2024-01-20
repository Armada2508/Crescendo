package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drive;
import frc.robot.lib.Encoder;
import frc.robot.lib.drive.DriveCommand;
import frc.robot.lib.util.Util;

public class DriveSubsystem extends SubsystemBase {

    private final TalonFX talonL = new TalonFX(Drive.LID);
    private final TalonFX talonR = new TalonFX(Drive.RID);
    private final TalonFX talonLF = new TalonFX(Drive.LFID);
    private final TalonFX talonRF = new TalonFX(Drive.RFID);
    private final PigeonIMU pigeon = new PigeonIMU(Drive.pigeonID);
    private final PIDController turnPIDController = new PIDController(Drive.turnPIDConfig.kP, Drive.turnPIDConfig.kI, Drive.turnPIDConfig.kD);
    private DifferentialDrivePoseEstimator poseEstimator;
    private final Supplier<Optional<EstimatedRobotPose>> visionEstimatedPose; 
    private final Field2d field2d = new Field2d();
    
    public DriveSubsystem(Supplier<Optional<EstimatedRobotPose>> visionEstimator) {
        this.visionEstimatedPose = visionEstimator;
        turnPIDController.setTolerance(0.5); // degrees
        configTalons();
        SmartDashboard.putData("Field", field2d);
    }

    @Override
    public void periodic() {
        if (pigeon.getState() != PigeonState.Ready) return;
        Optional<EstimatedRobotPose> visionPose = visionEstimatedPose.get();
        if (poseEstimator == null) { // Initialize field pose
            if (visionPose.isPresent()) {
                poseEstimator = new DifferentialDrivePoseEstimator(Drive.diffKinematics, getAngle(), getLeftPosition(), getRightPosition(), visionPose.get().estimatedPose.toPose2d());
            }
            return;
        }
        poseEstimator.update(getAngle(), getLeftPosition(), getRightPosition());
        if (visionPose.isPresent()) {
            poseEstimator.addVisionMeasurement(visionPose.get().estimatedPose.toPose2d(), visionPose.get().timestampSeconds
            /*,VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(45))*/);
        }   
        // System.out.println(getPose());
        field2d.setRobotPose(getFieldPose());
    }

    private void configTalons() {
        pigeon.configFactoryDefault();
        pigeon.setYaw(0);
        Util.factoryResetTalons(talonL, talonR, talonLF, talonRF);
        Util.brakeMode(talonL, talonR, talonLF, talonRF);
        talonL.setPosition(0);
        talonR.setPosition(0);
        talonLF.setControl(new StrictFollower(talonL.getDeviceID()));
        talonRF.setControl(new StrictFollower(talonR.getDeviceID()));
        talonLF.setInverted(true);
        talonRF.setInverted(true);
        talonL.getConfigurator().apply(Drive.slot0ConfigMotionMagic);
        talonR.getConfigurator().apply(Drive.slot0ConfigMotionMagic);
    }

    /**
     * 
     * @param velocity meters/second
     * @param acceleration meters/second^2
     */
    private void configMotionMagic(double velocity, double acceleration) {
        MotionMagicConfigs config = new MotionMagicConfigs();
        // Time period is 1s
        config.MotionMagicCruiseVelocity = Encoder.fromDistance(velocity, Drive.gearRatio, Drive.wheelDiameter);
        config.MotionMagicAcceleration = Encoder.fromDistance(acceleration, Drive.gearRatio, Drive.wheelDiameter);
        talonL.getConfigurator().apply(config);
        talonR.getConfigurator().apply(config);
    }

    /**
     * Drives the robot a certain distance relative to itself using motion magic
     * @param meters - distance to travel
     */
    private void driveDistance(double meters) {
        double distanceRots = Encoder.fromDistance(meters, Drive.gearRatio, Drive.wheelDiameter);
        MotionMagicVoltage request = new MotionMagicVoltage(distanceRots);
        talonL.setControl(request);
        talonR.setControl(request);
    }

    public void setSpeed(double leftSpeed, double rightSpeed) {
        talonL.set(leftSpeed);
        talonR.set(rightSpeed);
    }

    /**
     * 
     * @param distance meters relative to robot
     * @param velocity meters/second
     * @param acceleration meters/second^2
     * @return Command for the robot to drive distance using motion magic
     */
    public Command driveDistanceCommand(double distance, double velocity, double acceleration) {
        final double deadbandRotations = Encoder.fromDistance(Units.inchesToMeters(0.5), Drive.gearRatio, Drive.wheelDiameter); 
        return runOnce(() -> {
            configMotionMagic(velocity, acceleration);
            driveDistance(distance);
        })
        .andThen(Commands.waitUntil(() -> Util.inRange(talonL.getPosition().getValueAsDouble() - talonL.getClosedLoopReference().getValueAsDouble(), deadbandRotations)))
        .finallyDo(this::stop);
    }

    /**
     * 
     * @param angle - degrees relative to front of robot 
     * @return Command for the robot to turn amount of degrees relative to front of robot
     */
    public Command turnCommand(double angle) {
        turnPIDController.reset();
        turnPIDController.setSetpoint(pigeon.getYaw() + angle);
        return runEnd(() -> {
            double speed = turnPIDController.calculate(pigeon.getYaw());
            speed = MathUtil.clamp(speed, -Drive.maxTurnSpeed, Drive.maxTurnSpeed);
            setSpeed(speed, -speed);
        }, this::stop).until(turnPIDController::atSetpoint);
    }

    public void stop() {
        talonL.setControl(new NeutralOut());
        talonR.setControl(new NeutralOut());
    }

    private double getLeftPosition() {
        return Encoder.toDistance(talonL.getPosition().getValueAsDouble(), Drive.gearRatio, Drive.wheelDiameter);
    }

    private double getRightPosition() {
        return Encoder.toDistance(talonR.getPosition().getValueAsDouble(), Drive.gearRatio, Drive.wheelDiameter);
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(pigeon.getYaw()); //? I think this is right
    }

    public Pose2d getFieldPose() {
        return (poseEstimator == null) ? new Pose2d() : poseEstimator.getEstimatedPosition();
    }

    public Command joystickDriveCommand(DoubleSupplier joystickSpeed, DoubleSupplier joystickTurn, DoubleSupplier joystickTrim, BooleanSupplier joystickSlow) {
        return new DriveCommand(joystickSpeed, joystickTurn, joystickTrim, joystickSlow, Drive.joystickDriveConfig, this::setSpeed, this::stop, this);
    }

}
