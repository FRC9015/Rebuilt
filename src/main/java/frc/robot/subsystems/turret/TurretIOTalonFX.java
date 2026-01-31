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

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.turretConstants;

/** the. */
public class TurretIOTalonFX implements TurretIO {

  private final TalonFX elevatorMotor;
  private final CANcoder turretEncoderNoRatio;
  private final CANcoder turretEncoderFinalRatio;

  private final StatusSignal<Angle> encoderNoRatioPositionSignal;
  private final StatusSignal<Angle> encoderFinalRatioPositionSignal;
  private final StatusSignal<Voltage> motorAppliedVoltsSignal;
  private final StatusSignal<Current> motorCurrentSignal;
  private final StatusSignal<Angle> motorPosition;

  private final Debouncer encoderConnectedDebounce = new Debouncer(0.5);
  private final NeutralOut neutralOut = new NeutralOut();
  private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.5);

  SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs =
      new SoftwareLimitSwitchConfigs()
          .withForwardSoftLimitEnable(true)
          .withForwardSoftLimitThreshold(turretConstants.maxRotation)
          .withReverseSoftLimitEnable(true)
          .withReverseSoftLimitThreshold(turretConstants.minRoation);

  /**
   * Constructs an TurretIOTalonFX.
   *
   * @param motorID The ID of the motor.
   * @param followMotorID The ID of the second motor.
   * @param encoderId The ID of the encoder.
   */
  public TurretIOTalonFX(int motorID,int encoderId1,int encoderID2) {
    elevatorMotor = new TalonFX(motorID);
    turretEncoderNoRatio = new CANcoder(encoderId1);
    turretEncoderFinalRatio = new CANcoder(encoderID2);

    // Configure the motor
    TalonFXConfiguration motorConfig =
        new TalonFXConfiguration()
            .withSoftwareLimitSwitch(softwareLimitSwitchConfigs)
            .withMotionMagic(turretConstants.MOTION_MAGIC_CONFIGS)
            .withSlot0(turretConstants.SLOT0_CONFIGS)
            .withFeedback(turretConstants.FEEDBACK_CONFIGS)
            .withClosedLoopRamps(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.1));
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    elevatorMotor.getConfigurator().apply(motorConfig);
    // Configure the encoder

    //MIGHT NEED TO CHANGE LATER NOT 100% ON DIRECTION
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    turretEncoderNoRatio.getConfigurator().apply(encoderConfig);
    turretEncoderFinalRatio.getConfigurator().apply(encoderConfig);

    // Signals
    encoderNoRatioPositionSignal = turretEncoderNoRatio.getPosition();
    encoderFinalRatioPositionSignal = turretEncoderFinalRatio.getPosition();
    motorAppliedVoltsSignal = elevatorMotor.getMotorVoltage();
    motorCurrentSignal = elevatorMotor.getStatorCurrent();
    motorPosition = elevatorMotor.getPosition();
    

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, encoderNoRatioPositionSignal, encoderFinalRatioPositionSignal,motorAppliedVoltsSignal, motorCurrentSignal, motorPosition);
    turretEncoderNoRatio.optimizeBusUtilization();
    elevatorMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // Refresh signals
    StatusCode encoderStatus =
        BaseStatusSignal.refreshAll(
            encoderNoRatioPositionSignal, motorAppliedVoltsSignal, motorCurrentSignal, motorPosition,encoderFinalRatioPositionSignal);

    // Update elevator inputs
    inputs.turretEncoderNoRatioConnected = encoderConnectedDebounce.calculate(encoderStatus.isOK());
    inputs.turretEncoderNoRatioPosition = encoderNoRatioPositionSignal.getValueAsDouble();
    inputs.turretEncoderFinalRatioPosition = encoderFinalRatioPositionSignal.getValueAsDouble();
    inputs.turretAppliedVolts = motorAppliedVoltsSignal.getValueAsDouble();
    inputs.turretCurrentAmps = motorCurrentSignal.getValueAsDouble();
    inputs.turretMotorPosition = motorPosition.getValueAsDouble();
  }

  @Override
  public void setturretPosition(double value) {
    // elevatorMotor.setControl(voltageOut.withOutput(MathUtil.clamp(value, -12, 12)));
    if (value > turretConstants.maxRotation) {
      value = turretConstants.maxRotation;
    }
    if (value < turretConstants.minRoation) {
      value = turretConstants.minRoation;
    }
    elevatorMotor.setControl(motionMagicVoltage.withPosition(value));
  }

  @Override
  public void stop() {
    elevatorMotor.setControl(neutralOut);
  }

  @Override
  public void setBrakeMode() {
    elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void setCoastMode() {
    elevatorMotor.setNeutralMode(NeutralModeValue.Coast);
  }
}
