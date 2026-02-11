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

import org.littletonrobotics.junction.AutoLog;

/** Interface for the Indexer Input/Output operations. */
public interface IndexerIO {
  /** Class representing the inputs for the Indexer. */
  @AutoLog
  public static class IndexerIOInputs {

    // Fields representing the end effector state and inputs
    public double indexerVelocity = 0.0;
    public double indexerAppliedVolts = 0.0;
    public double indexerCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IndexerIOInputs inputs) {}

  /** Stop indexer from running. */
  default void stop() {}

  /** Enable or disable brake mode on the end effector motor. */
  default void setBrakeMode(boolean enable) {}

  /**
   * Sets the voltage of the indexer motor.
   *
   * @param voltage The desired voltage.
   */
  default void setVoltage(double voltage) {}
}
