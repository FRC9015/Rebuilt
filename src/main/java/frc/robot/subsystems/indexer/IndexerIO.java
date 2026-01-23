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

/** Interface for the End Effector Input/Output operations. */
public interface IndexerIO {
  /** Class representing the inputs for the End Effector. */
  @AutoLog
  public static class IndexerIOInputs {

    // Fields representing the end effector state and inputs
    public double IndexerAppliedVolts = 0.0;
    public double IndexerCurrentAmps = 0.0;
    public double IndexerRPM = 0.0;
    public boolean IndexerEncoderConnected = false;
    public boolean JamDetected = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IndexerIOInputs inputs) {}

  /** Stop indexer from running. */
  default void stop() {}

  /** Enable or disable brake mode on the end effector motor. */
  default void setBrakeMode(boolean enable) {}

  /**
   * Sets the RPM of the indexer motor.
   *
   * @param rpm The desired RPM.
   */
  default void setRPM(double rpm) {}
}