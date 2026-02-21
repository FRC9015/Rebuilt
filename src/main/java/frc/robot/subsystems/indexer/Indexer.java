// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** The Indexer subsystem controls the indexer mechanism. */
public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  private final Alert encoderDisconnectedAlert;
  private final Alert jamAlert;

  private static final double jamCurrentAmps = 30.0;
  private static final double jamRPMThreshold = 50.0;
  private static final int jamCyclesThreshold = 10; // ~0.2s at 50Hz

  private static boolean highCurrent;
  private static boolean lowSpeed;
  private static boolean isRunning;

  @AutoLogOutput private int jamCycles = 0;

  /**
   * Constructs an Indexer subsystem.
   *
   * @param io The input/output interface for the indexer.
   */
  public Indexer(IndexerIO io) {
    this.io = io;
    encoderDisconnectedAlert = new Alert("Indexer encoder disconnected!", AlertType.kError);
    jamAlert = new Alert("Jam detected!", AlertType.kInfo);
  }

  /** Periodically updates the indexer's state and logs inputs. */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);

    highCurrent = inputs.indexerCurrentAmps >= jamCurrentAmps;
    lowSpeed = Math.abs(inputs.indexerVelocity) <= jamRPMThreshold;
    isRunning = Math.abs(inputs.indexerAppliedVolts) > 0.1;

    if (highCurrent && lowSpeed && isRunning) {
      jamCycles++;
    } else {
      jamCycles = 0;
    }
    
    jamAlert.set(isJamDetected());
  }

  /**
   * Sets the indexer's applied voltage.
   *
   * @param voltage The desired voltage for the indexer.
   */
  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
    Logger.recordOutput("Indexer/setVoltage", voltage);
  }

  /** Stops the indexer. */
  public void stop() {
    io.stop();
    Logger.recordOutput("Indexer/Stopped", true);
  }

  /**
   * Enables or disables brake mode for the indexer motors.
   *
   * @param enable True to enable brake mode, false for coast mode.
   */
  public void setBrakeMode(boolean enable) {
    io.setBrakeMode(enable);
    Logger.recordOutput("Indexer/BrakeMode", enable);
  }

  /**
   * Checks whether jam is currently detected and increments appropretly.
   *
   * @return True if jam is detected, false otherwise.
   */
  public boolean isJamDetected() {
    return jamCycles >= jamCyclesThreshold;
  }

  /**
   * Returns the current RPM of the indexer.
   *
   * @return The RPM of the indexer.
   */
  public double getRPM() {
    return inputs.indexerVelocity;
  }

  /**
   * Returns the current voltage applied to the indexer motors.
   *
   * @return The applied voltage.
   */
  public double getAppliedVolts() {
    return inputs.indexerAppliedVolts;
  }

  /**
   * Returns the current drawn by the indexer motors.
   *
   * @return The current in amps.
   */
  public double getCurrentAmps() {
    return inputs.indexerCurrentAmps;
  }

  /**
   * Runs the indexer at the specified voltage.
   *
   * @param voltage Voltage provided to the motor.
   * @return A command that runs the indexer.
   */
  public Command runIndexer(double voltage) {
    return this.startEnd(() -> io.setVoltage(voltage), () -> stop());
  }

  /**
   * Unjams the indexer by running it in reverse for 0.5 seconds.
   *
   * @return A command that unjams the indexer.
   */
  public Command unjam() {
    return this.runEnd(() -> io.setVoltage(-4.0), () -> stop()).withTimeout(0.5);
  }

  /**
   * Runs the indexer at the specified voltage but automatically unjams.
   *
   * @param voltage Voltage provided to the motor.
   * @return A command that runs the indexer with auto unjam functionality.
   */
  public Command runIndexerWithAutoUnjam(double voltage) {
    return runIndexer(voltage)
        .until(this::isJamDetected)
        .andThen(unjam())
        .andThen(this.runIndexerWithAutoUnjam(voltage));
  }
}
