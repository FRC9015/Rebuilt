package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private double setpoint = 0.0;

  private final ShooterIO io;

  private double idleSpeed = 0.0;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  // Minimum Value of speedValue: -100 RPS
  // Maximum Value of speedValue: 100 RPS

  public void setShooterSpeed(double speedValue) {
    io.setFlyWheelSpeed(speedValue);
  }

  public void incrementFlyWheelSpeed(double value) {
    setpoint += (1 * value);
  }

  public void setShooterReverseSpeed(double speedValue) {

    io.setFlyWheelSpeed(-speedValue);
  }

  public void setKickerSpeed(double speedValue) {
    io.setKickerSpeed(speedValue);
  }

  public void stopKicker() {
    io.stopKicker();
  }

  public void setSetpoint(double setpoint) {
    this.setpoint = setpoint;
  }

  public void setKickerSpeedReverse(double speedValue) {
    io.setKickerSpeed(-speedValue);
  }

  public Command setKickerSpeedCommand(double speedValue) {
    return this.startEnd(() -> this.setKickerSpeed(speedValue), () -> io.stopKicker());
  }

  public Command setKickerSpeedReverseCommand(double speedValue) {
    return this.run(() -> this.setKickerSpeedReverse(speedValue));
  }

  public Command runShooterAtSpeed(double speed) {
    return this.startEnd(
        () -> {
          this.setShooterSpeed(speed);
        },
        () -> {
          io.stopFlywheels();
        });
  }

  public Command runShooterAtReverseSpeed(double speed) {
    Logger.recordOutput("Shooter/Speed", speed);
    return this.startEnd(() -> this.setShooterReverseSpeed(speed), () -> io.stopFlywheels());
  }

  public Command stopFlywheels() {
    return this.run(() -> io.stopFlywheels());
  }

  public Command incrementShooterCommand(double value) {
    return this.runOnce(() -> incrementFlyWheelSpeed(value));
  }

  public boolean returnShooterAtSetpoint() {
    return inputs.flywheelAtSpeed;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    if (DriverStation.isTest()) {
      io.setFlyWheelSpeed(setpoint);
      Logger.recordOutput("ShooterTest/setpoint", setpoint);
    }
  }
}
