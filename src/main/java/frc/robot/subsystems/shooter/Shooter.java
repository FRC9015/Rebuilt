package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

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

  public void setShooterReverseSpeed(double speedValue) {

    io.setFlyWheelSpeed(-speedValue);
  }

  public void setKickerSpeed(double speedValue) {
    io.setKickerSpeed(speedValue);
  }

  public void setKickerSpeedReverse(double speedValue) {
    io.setKickerSpeed(-speedValue);
  }

  public Command setKickerSpeedCommand(double speedValue) {
    return this.run(() -> this.setKickerSpeed(speedValue));
  }

  public Command setKickerSpeedReverseCommand(double speedValue) {
    return this.run(() -> this.setKickerSpeedReverse(speedValue));
  }

  public Command runShooterAtSpeed(double speed) {
    Logger.recordOutput("Shooter/Speed", speed);
    return this.startEnd(() -> {
      this.setShooterSpeed(speed);
      
      if (inputs.flywheelAtSpeed) {
        this.setKickerSpeed(speed);
      }
    }, () -> io.stopFlywheels());
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

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }
}
