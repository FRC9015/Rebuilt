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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.IndexerConstants;

/** IO implementation for the Indexer subsystem using a TalonFX motor controller. */
public class IndexerIOTalonFX implements IndexerIO {

  private final TalonFX motor1, motor2;
  private final StatusSignal<Voltage> appliedVoltsSignalMotor1, appliedVoltsSignalMotor2;
  private final StatusSignal<Current> currentSignalMotor1, currentSignalMotor2;

  private final double defaultCurrentLimit = 40.0;
  private final double maxVoltage = 12.0;

  private double indexerSetpoint = 0.0;

  private MotionMagicVelocityVoltage indexerVelocity =
      new MotionMagicVelocityVoltage(0.5).withSlot(0);

  public IndexerIOTalonFX(
      int motorId1, int motorid2) { // , int canRangeID1, int canRangeID2, int canRangeID3
    motor1 = new TalonFX(motorId1);
    motor2 = new TalonFX(motorid2);
    // Configure motor
    TalonFXConfiguration motorConfig =
        new TalonFXConfiguration()
            .withSlot0(IndexerConstants.SLOT0_CONFIGS)
            .withFeedback(IndexerConstants.FEEDBACK_CONFIGS)
            .withMotionMagic(IndexerConstants.MOTION_MAGIC_CONFIGS);
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfig.CurrentLimits.StatorCurrentLimit = defaultCurrentLimit;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Configure the integrated encoder (default settings should work)
    motor1.getConfigurator().apply(motorConfig);
    motor2.getConfigurator().apply(motorConfig);

    // Use the built-in relative encoder of the TalonFX
    appliedVoltsSignalMotor1 = motor1.getMotorVoltage();
    currentSignalMotor1 = motor1.getStatorCurrent();

    appliedVoltsSignalMotor2 = motor2.getMotorVoltage();
    currentSignalMotor2 = motor2.getStatorCurrent();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    // Refresh signals
    BaseStatusSignal.refreshAll(
        appliedVoltsSignalMotor1,
        currentSignalMotor1,
        appliedVoltsSignalMotor2,
        currentSignalMotor2);
    // Update inputs
    inputs.indexerAppliedVoltsMotor1 = appliedVoltsSignalMotor1.getValueAsDouble();
    inputs.indexerCurrentAmpsMotor1 = currentSignalMotor1.getValueAsDouble();
    inputs.indexerAppliedVoltsMotor2 = appliedVoltsSignalMotor2.getValueAsDouble();
    inputs.indexerCurrentAmpsMotor2 = currentSignalMotor2.getValueAsDouble();
    inputs.indexerVelocityMotor1 = motor1.getVelocity().getValueAsDouble();
    inputs.indexerVelocityMotor2 = motor2.getVelocity().getValueAsDouble();
    inputs.indexerSetpoint = indexerSetpoint;
  }

  @Override
  public void stop() {
    motor1.stopMotor();
    motor2.stopMotor();
  }

  @Override
  public void setBrakeMode(boolean enable) {
    motor1.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    motor2.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setIndexerSpeed(double speed) {
    indexerSetpoint = speed;
    motor1.setControl(indexerVelocity.withVelocity(speed));
    motor2.setControl(indexerVelocity.withVelocity(speed));
  }
}
