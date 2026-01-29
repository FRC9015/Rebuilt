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
import org.littletonrobotics.junction.Logger;

/** The Indexer subsystem controls the indexer mechanism. */
public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  private final Alert encoderDisconnectedAlert;
  private final Alert jamAlert;

  /**
   * Constructs an Indexer subsystem.
   *
   * @param io The input/output interface for the indexer.
   */
  public Indexer(IndexerIO io) {
    this.io = io;
    encoderDisconnectedAlert = new Alert("Indexer encoder disconnected!", AlertType.kError);
    JamAlert = new Alert("Jam detected!", AlertType.kInfo);
  }

  /** Periodically updates the indexer's state and logs inputs. */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);

    // Update alerts
    encoderDisconnectedAlert.set(!inputs.IndexerEncoderConnected);
    JamAlert.set(inputs.JamDetected);
  }

  /**
   * Sets the indexer's RPM.
   *
   * @param voltage The desired RPM for the indexer.
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
   * Returns whether jam is currently detected.
   *
   * @return True if jam is detected, false otherwise.
   */
  public boolean isJamDetected() {
    return inputs.JamDetected;
  }

  /**
   * Returns the current RPM of the indexer.
   *
   * @return The RPM of the indexer.
   */
  public double getRPM() {
    return inputs.IndexerRPM;
  }

  /**
   * Returns the current voltage applied to the indexer motors.
   *
   * @return The applied voltage.
   */
  public double getAppliedVolts() {
    return inputs.IndexerAppliedVolts;
  }

  /**
   * Returns the current drawn by the indexer motors.
   *
   * @return The current in amps.
   */
  public double getCurrentAmps() {
    return inputs.IndexerCurrentAmps;
  }

  /**
   * Runs the indexer at the specified RPM.
   *
   * @param voltage Voltage provided to the motor.
   * @return A command that runs the indexer.
   */
  public Command runIndexer(double voltage) {
    return this.startEnd(() -> setVoltage(-voltage), () -> stop());
  }

  /**
   * idk auto stuff copy pasted from reefscape end effector public Command runIndexerAuto(double
   * voltage) { return this.run(() -> setVoltage(-voltage)); }
   *
   * <p>public Command runIndexerAutoCommand() { return new SequentialCommandGroup( new
   * InstantCommand(this::autoIndexerVoltage), new WaitCommand(0.25), new
   * InstantCommand(this::stop)); }
   *
   * <p>public Command runIndexerAutoCommandL1() { return new SequentialCommandGroup( new
   * InstantCommand(this::autoIndexerVoltageL1), new WaitCommand(0.4), new
   * InstantCommand(this::stop)); }
   *
   * <p>private void autoIndexerVoltage() { io.setRPM(-4); }
   *
   * <p>private void autoIndexerVoltageL1() { io.setRPM(-4); } /** Runs the indexer in reverse at
   * the specified RPM.
   *
   * @param voltage Voltage provided to the motor.
   * @return A command that runs the indexer in reverse.
   */
  public Command runIndexerReverse(double voltage) {
    return this.startEnd(() -> setVoltage(voltage), () -> stop());
  }
}
