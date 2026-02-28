package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.ObjectDetection;
import org.littletonrobotics.junction.Logger;
import frc.robot.Constants.VisionAlignConstants;

public class VisionAlign extends Command {

    private final Drive drive;
    private final ObjectDetection vision;

    private final PIDController xController = new PIDController(VisionAlignConstants.X_KP, VisionAlignConstants.X_KI, VisionAlignConstants.X_KD);
    private final PIDController yController = new PIDController(VisionAlignConstants.Y_KP, VisionAlignConstants.Y_KI, VisionAlignConstants.Y_KD);
    private final PIDController rotationController = new PIDController(VisionAlignConstants.ROTATION_KP, VisionAlignConstants.ROTATION_KI, VisionAlignConstants.ROTATION_KD);

    private Translation2d goalTranslation = new Translation2d();
    private boolean hasGoal = false;

    public VisionAlign(Drive drive, ObjectDetection vision) {
        this.drive = drive;
        this.vision = vision;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        rotationController.enableContinuousInput(-180, 180);
        rotationController.setTolerance(VisionAlignConstants.ROTATION_TOLERANCE_DEGREES);
        xController.setTolerance(VisionAlignConstants.XY_TOLERANCE_METERS);
        yController.setTolerance(VisionAlignConstants.XY_TOLERANCE_METERS);
        hasGoal = false;
    }

    @Override
    public void execute() {

        if (!hasGoal) {
            if (!vision.hasTarget()) {
                drive.runVelocity(new ChassisSpeeds());
                return;
            }

            goalTranslation = vision.getLastTranslation();
            hasGoal = true;
        }

        double distance = goalTranslation.getNorm();

        if (distance < VisionAlignConstants.DISTANCE_TOLERANCE_METERS) {
            drive.runVelocity(new ChassisSpeeds());
            return;
        }

        double vx = xController.calculate(goalTranslation.getX(), 0.0);
        double vy = yController.calculate(goalTranslation.getY(), 0.0);

        double omega =
            rotationController.calculate(vision.getYawDeg(), 0.0);

        ChassisSpeeds speeds = new ChassisSpeeds(
            vx,
            vy,
            Math.toRadians(omega)
        );

        drive.runVelocity(speeds);

        Logger.recordOutput("VisionDrive/GoalX", goalTranslation.getX());
        Logger.recordOutput("VisionDrive/GoalY", goalTranslation.getY());
        Logger.recordOutput("VisionDrive/Distance", distance);
    }

    @Override
    public boolean isFinished() {
        return hasGoal
            && xController.atSetpoint()
            && yController.atSetpoint()
            && rotationController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drive.runVelocity(new ChassisSpeeds());
    }
}