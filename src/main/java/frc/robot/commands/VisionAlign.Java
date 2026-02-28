package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.ObjectDetection;
import org.littletonrobotics.junction.Logger;

public class VisionDriveToTarget extends Command {

    private final Drive drive;
    private final ObjectDetection vision;

    private final PIDController xController = new PIDController(2.5, 0, 0.02);
    private final PIDController yController = new PIDController(2.5, 0, 0.02);
    private final PIDController rotationController = new PIDController(4.0, 0, 0.02);

    private Translation2d goalTranslation = new Translation2d();
    private boolean hasGoal = false;

    public VisionDriveToTarget(Drive drive, ObjectDetection vision) {
        this.drive = drive;
        this.vision = vision;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        rotationController.enableContinuousInput(-180, 180);
        rotationController.setTolerance(1.0);
        xController.setTolerance(0.1);
        yController.setTolerance(0.1);
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

        if (distance < 0.05) {
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