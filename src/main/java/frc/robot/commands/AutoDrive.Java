package frc.robot.commands;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AutoDrive extends Command {
    private final double ROTATION_P = 4;
    private final double ROTATION_I = 0;
    private final double ROTATION_D = 0.02;
    private final double XYCONTROLLERP = 3;
    private final double XYCONTROLLERI = 0.0;
    private final double XYCONTROLLERD = 0.02;
    private final double TIME_LIMIT = 3;
    private final double MINIMUM_VELOCITY = 0.3;
    private final double MINIMUM_DISTANCE = 0.04;
    private PIDController rotationController = new PIDController(ROTATION_P, ROTATION_I, ROTATION_D);
    private PIDController xController = new PIDController(XYCONTROLLERP, XYCONTROLLERI, XYCONTROLLERD);
    private PIDController yController = new PIDController(XYCONTROLLERP, XYCONTROLLERI, XYCONTROLLERD);
    private Supplier<Pose2d> targetPose;
    private Pose2d flippedPose, targetPose2d;
    private Drive drive;
    private Supplier<DriveStation.Alliance> alliance;

    private final Timer timer = new Timer();

    public AutoDrive(
        Supplier<Pose2d> desiredPose, Drive drive, Supplier<DriverStation.Alliance> alliance) {
        this.drive = drive;
        this.targetPose = desiredPose;
        this.alliance = alliance;
    }
    @Override
    public void initialize() {
        flippedPose = FlippingUtil.flipFieldPose(targetPose.get());
        this.targetPose2d =
            alliance.get() == DriverStation.Alliance.Red ? flippedPose : targetPose.get();
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(Units.degreesToRadians(1));
        yController.setTolerance(Units.inchesToMeters(0.4));
        xController.setTolerance(Units.inchesToMeters(0.4));
        Logger.recordOutput("AutoDrive/alliance?", alliance.get());

        timer.reset();
        timer.start();
    }
    @Override
    public void execute() {
        Pose2d currentPose = drive.getPredictedPose();

        double rotationalVelocity =
            rotationController.calculate(
                currentPose.getRotation().getRadians(), targetPose2d.getRotation().getRadians());
        double yVelocity = yController.calculate(currentPose.getY(), targetPose2d.getY());
        double xVelocity = xController.calculate(currentPose.getX(), targetPose2d.getX());

        ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();

        robotRelativeSpeeds.vxMetersPerSecond = xVelocity;
        robotRelativeSpeeds.vyMetersPerSecond = yVelocity;
        robotRelativeSpeeds.omegaRadiansPerSecond = rotationalVelocity;

        ChassisSpeeds field =
            ChassisSpeeds.fromFieldRelativeSpeeds(robotRelativeSpeeds, currentPose.getRotation());
        drive.runVelocity(field);

        Logger.recordOutput("AutoDrive/targetPose", targetPose.get());
        Logger.recordOutput("AutoDrive/robotrelativespeeds", field);
        Logger.recordOutput("AutoDrive/flippedPose", flippedPose);
    }
    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        double distance =
            Math.hypot(
                drive.getPredictedPose().getX() - targetPose2d.getX(),
                drive.getPredictedPose().getY() - targetPose2d.getY());
        return
        (rotationController.atSetpoint() && yController.atSetpoint() && xController.atSetpoint())
            || timer.hasElapsed(TIME_LIMIT)
            || drive.getVelocityMetersPerSec() < MINIMUM_VELOCITY && distance < MINIMUM_DISTANCE;
    }
}