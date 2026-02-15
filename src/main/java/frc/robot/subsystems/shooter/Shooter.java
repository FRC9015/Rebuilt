package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final ShooterIO io;
  private final double defaultHoodPosition = 0.0;

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
    // this.setDefaultCommand(setHoodPosition(defaultHoodPosition));
  }

  // Minimum Value of speedValue: -512.0
  // Maximum Value of speedValkue: 511.998046875

  public void setShooterSpeed(double speedValue) {
    io.setFlyWheelSpeed(speedValue);
  }

  public void setShooterReverseSpeed(double speedValue) {

    io.setFlyWheelSpeed(-speedValue);
  }

  public void setShooterVoltage(double volts) {
    io.setFlyWheelVoltage(volts);
  }

  public Command runShooterAtVoltage(double volts) {
    Logger.recordOutput("Shooter/Voltage", volts);
    return this.startEnd(() -> this.setShooterVoltage(volts), () -> io.stopFlywheels());
  }

  public Command runShooterAtSpeed(double speed) {
    Logger.recordOutput("Shooter/Speed", speed);
    return this.startEnd(() -> this.setShooterSpeed(speed), () -> io.stopFlywheels());
  }

  public Command runShooterAtReverseSpeed(double speed) {
    Logger.recordOutput("Shooter/Speed", speed);
    return this.startEnd(() -> this.setShooterReverseSpeed(speed), () -> io.stopFlywheels());
  }

  public Command stopFlywheels() {
    return this.run(() -> io.stopFlywheels());
  }

  public Command stopHood() {
    return this.run(() -> io.stopHood());
  }

  public Command setHoodPosition(double position) {
    return this.run(() -> io.setHoodPosition(position));
  }

  public Command setHoodZero() {
    return this.startEnd(() -> io.setHoodZero(), () -> io.stopHood());
  }

  public Command setHoodZeroReverse() {
    return this.startEnd(() -> io.setHoodZeroReverse(), () -> io.stopHood());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }
}
