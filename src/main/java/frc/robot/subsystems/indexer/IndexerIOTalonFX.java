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

  private final TalonFX hotDogMotor, ballTunnelMotor;
  private final StatusSignal<Voltage> appliedVoltsSignalMotor1, appliedVoltsSignalMotor2;
  private final StatusSignal<Current> currentSignalMotor1, currentSignalMotor2;

  private final double defaultCurrentLimit = 40.0;
  private final double maxVoltage = 12.0;

  private double indexerSetpoint = 0.0;

  private MotionMagicVelocityVoltage indexerVelocity =
      new MotionMagicVelocityVoltage(0.5).withSlot(0);

  private MotionMagicVelocityVoltage tunnelVelocity =
      new MotionMagicVelocityVoltage(0.5).withSlot(0);

  public IndexerIOTalonFX(
      int motorId1, int motorid2) { // , int canRangeID1, int canRangeID2, int canRangeID3
    hotDogMotor = new TalonFX(motorId1);
    ballTunnelMotor = new TalonFX(motorid2);

    // Configure motor
    TalonFXConfiguration motorConfig =
        new TalonFXConfiguration()
            .withSlot0(IndexerConstants.INDEXER_SLOT0_CONFIGS)
            .withFeedback(IndexerConstants.INDEXER_FEEDBACK_CONFIGS)
            .withMotionMagic(IndexerConstants.INDEXER_MOTION_MAGIC_CONFIGS);
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfig.CurrentLimits.StatorCurrentLimit = defaultCurrentLimit;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    TalonFXConfiguration ballTunnelConfig =
        new TalonFXConfiguration()
            .withSlot0(IndexerConstants.TUNNEL_SLOT0_CONFIGS)
            .withFeedback(IndexerConstants.TUNNEL_FEEDBACK_CONFIGS)
            .withMotionMagic(IndexerConstants.TUNNEL_MOTION_MAGIC_CONFIGS);
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfig.CurrentLimits.StatorCurrentLimit = defaultCurrentLimit;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Configure the integrated encoder (default settings should work)
    hotDogMotor.getConfigurator().apply(motorConfig);
    ballTunnelMotor.getConfigurator().apply(ballTunnelConfig);

    // Use the built-in relative encoder of the TalonFX
    appliedVoltsSignalMotor1 = hotDogMotor.getMotorVoltage();
    currentSignalMotor1 = hotDogMotor.getStatorCurrent();

    appliedVoltsSignalMotor2 = ballTunnelMotor.getMotorVoltage();
    currentSignalMotor2 = ballTunnelMotor.getStatorCurrent();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    // Refresh signals
    BaseStatusSignal.refreshAll(appliedVoltsSignalMotor1, currentSignalMotor1);
    // Update inputs
    inputs.indexerAppliedVoltsMotor1 = appliedVoltsSignalMotor1.getValueAsDouble();
    inputs.indexerCurrentAmpsMotor1 = currentSignalMotor1.getValueAsDouble();
    inputs.indexerVelocityMotor1 = hotDogMotor.getVelocity().getValueAsDouble();
    inputs.tunnelAppliedVoltsMotor2 = appliedVoltsSignalMotor2.getValueAsDouble();
    inputs.tunnelCurrentAmpsMotor2 = currentSignalMotor2.getValueAsDouble();
    inputs.tunnelVelocityMotor2 = ballTunnelMotor.getVelocity().getValueAsDouble();
    inputs.indexerSetpoint = indexerSetpoint;
    inputs.tunnelStall =
        ballTunnelMotor.getMotorStallCurrent().getValueAsDouble()
                <= currentSignalMotor2.getValueAsDouble()
            || (inputs.indexerSetpoint != 0 && inputs.tunnelVelocityMotor2 < 3.0);
    inputs.indexerStall =
        hotDogMotor.getMotorStallCurrent().getValueAsDouble()
                <= currentSignalMotor1.getValueAsDouble()
            || (inputs.indexerSetpoint != 0 && inputs.indexerVelocityMotor1 < 3.0);
  }

  @Override
  public void stop() {
    hotDogMotor.stopMotor();
    ballTunnelMotor.stopMotor();
    indexerSetpoint = 0;
  }

  @Override
  public void setBrakeMode(boolean enable) {
    hotDogMotor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setIndexerSpeed(double speed, double tunnel) {
    indexerSetpoint = speed;
    hotDogMotor.setControl(indexerVelocity.withVelocity(speed));
    ballTunnelMotor.setControl(tunnelVelocity.withVelocity(tunnel));
  }

  public void setIndexerVoltage(double voltage) {
    indexerSetpoint = voltage;
    hotDogMotor.setVoltage(voltage);
  }

  public void setBallTunnelSpeed(double speed) {
    ballTunnelMotor.setControl(tunnelVelocity.withVelocity(speed));
  }
}
