package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Subsystem that controls the robot's intake mechanism (roller + pivot). */
public class Intake extends SubsystemBase {
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakeIO io;

  private PIDController intakePIDController;

  private double kP = 0.0;
  private double kI = 0.0;
  private double kD = 0.0;
  private double toleranceMeters = 0.0;
  private double idleSpeed = 0.0;

  public Intake(IntakeIO io) {
    this.io = io;
    intakePIDController = new PIDController(kP, kI, kD);
    intakePIDController.setTolerance(toleranceMeters);
  }

  // Minimum Value of speedValue: -512.0
  // Maximum Value of speedValkue: 511.998046875

  public void setIntakeSpeed(double speedValue) {
    io.setIntakeSpeed(speedValue);
  }

  public void setIntakeReverseSpeed(double speedValue) {

    io.setIntakeSpeed(-speedValue);
  }

  public Command runIntakeAtSpeed(double speed) {
    Logger.recordOutput("Intake/Speed", speed);
    return this.startEnd(() -> this.setIntakeSpeed(speed), () -> this.setIntakeSpeed(idleSpeed));
  }

  public Command runIntakeAtReverseSpeed(double speed) {
    Logger.recordOutput("Intake/Speed", speed);
    return this.startEnd(
        () -> this.setIntakeReverseSpeed(speed), () -> this.setIntakeReverseSpeed(idleSpeed));
  }

  public Command runIntake() {
    return this.startEnd(() -> io.setRunning(true), () -> io.setRunning(false));
  }

  public Command stopIntake() {
    return this.run(() -> io.stop());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }
}
