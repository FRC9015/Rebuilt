package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final ShooterIO io;

  private PIDController shooterPIDController;

  private double kP = 0.0;
  private double kI = 0.0;
  private double kD = 0.0;
  private double toleranceMeters = 0.0;
  private double idleSpeed = 0.0;

  public Shooter(ShooterIO io) {
    this.io = io;
    shooterPIDController = new PIDController(kP, kI, kD);
    shooterPIDController.setTolerance(toleranceMeters);
  }

  // Minimum Value of speedValue: -512.0
  // Maximum Value of speedValkue: 511.998046875

  public void setShooterSpeed(double speedValue) {
    io.setFlyWheelSpeed(speedValue);
  }

  public void setShooterReverseSpeed(double speedValue) {

    io.setFlyWheelSpeed(-speedValue);
  }

  public Command runShooterAtSpeed(double speed) {
    Logger.recordOutput("Shooter/Speed", speed);
    return this.startEnd(() -> this.setShooterSpeed(speed), () -> this.setShooterSpeed(idleSpeed));
  }

  public Command runShooterAtReverseSpeed(double speed) {
    Logger.recordOutput("Shooter/Speed", speed);
    return this.startEnd(
        () -> this.setShooterReverseSpeed(speed), () -> this.setShooterSpeed(idleSpeed));
  }

  public Command stopFlywheels() {
    return this.run(() -> io.stopFlywheels());
  }

  public Command stopHood() {
    return this.run(() -> io.stopHood());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }
}
