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

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** IO implementation for the Indexer subsystem using a TalonFX motor controller. */
public class IndexerIOTalonFX implements IndexerIO {

  private final TalonFX motor1;
  private final StatusSignal<Voltage> appliedVoltsSignal;
  private final StatusSignal<Current> currentSignal;

  private final double defaultCurrentLimit = 45.0;
  private final double maxVoltage = 12.0;

  public IndexerIOTalonFX(int motorId1) { // , int canRangeID1, int canRangeID2, int canRangeID3
    motor1 = new TalonFX(motorId1);

    // Configure motor
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfig.CurrentLimits.StatorCurrentLimit = defaultCurrentLimit;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Configure the integrated encoder (default settings should work)
    motor1.getConfigurator().apply(motorConfig);

    // Use the built-in relative encoder of the TalonFX
    appliedVoltsSignal = motor1.getMotorVoltage();
    currentSignal = motor1.getStatorCurrent();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    // Update inputs
    inputs.indexerAppliedVolts = appliedVoltsSignal.getValueAsDouble();
    inputs.indexerCurrentAmps = currentSignal.getValueAsDouble();
  }

  @Override
  public void stop() {
    motor1.stopMotor();
  }

  @Override
  public void setBrakeMode(boolean enable) {
    motor1.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setVoltage(double voltage) {
    motor1.setVoltage(MathUtil.clamp(voltage, -maxVoltage, maxVoltage));
  }
}
