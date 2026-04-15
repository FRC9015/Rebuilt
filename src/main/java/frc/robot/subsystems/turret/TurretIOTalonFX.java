package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.TurretConstants;

public class TurretIOTalonFX implements TurretIO {

  private final TalonFX turretMotor;
  private final DigitalInput hallEffectSensor;

  private double setpointDegrees = 0.0;

  private final StatusSignal<Voltage> motorAppliedVoltsSignal;
  private final StatusSignal<Current> motorCurrentSignal;
  private final StatusSignal<edu.wpi.first.units.measure.Angle> motorPositionSignal;

  private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);
  private final VoltageOut voltageOut = new VoltageOut(0.0);

  public TurretIOTalonFX(int motorID, int hallEffectChannel) {
    turretMotor = new TalonFX(motorID);
    hallEffectSensor = new DigitalInput(hallEffectChannel);

    TalonFXConfiguration motorConfig =
        new TalonFXConfiguration()
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(TurretConstants.MAXROTATION)
                    .withReverseSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(TurretConstants.MINROTATION))
            .withMotionMagic(TurretConstants.MOTION_MAGIC_CONFIGS)
            .withSlot0(TurretConstants.SLOT0_CONFIGS)
            .withFeedback(TurretConstants.FEEDBACK_CONFIGS)
            .withClosedLoopRamps(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.1))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(60.0)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(60.0)
                    .withSupplyCurrentLimitEnable(true));

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    turretMotor.getConfigurator().apply(motorConfig);

    motorAppliedVoltsSignal = turretMotor.getMotorVoltage();
    motorCurrentSignal = turretMotor.getStatorCurrent();
    motorPositionSignal = turretMotor.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, motorAppliedVoltsSignal, motorCurrentSignal, motorPositionSignal);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    BaseStatusSignal.refreshAll(motorAppliedVoltsSignal, motorCurrentSignal, motorPositionSignal);

    // REV Hall Effect is Active Low (false when magnet present).
    // We invert it so "true" means "Magnet Detected".
    inputs.hallEffectTriggered = !hallEffectSensor.get();

    inputs.turretAppliedVolts = motorAppliedVoltsSignal.getValueAsDouble();
    inputs.turretCurrentAmps = motorCurrentSignal.getValueAsDouble();
    inputs.turretMotorPosition = motorPositionSignal.getValueAsDouble();
    inputs.turretSetpoint = setpointDegrees;

    // Position logic simplified: motor position IS the turret position now
    inputs.turretResolvedPosition = inputs.turretMotorPosition;
    inputs.turretResolvedPositionDegrees = inputs.turretMotorPosition * 360.0;

    inputs.turretError = (Math.abs(inputs.turretResolvedPositionDegrees - inputs.turretSetpoint));
  }

  @Override
  public void seedMotorPosition(double positionRotations) {
    turretMotor.setPosition(positionRotations);
  }

  @Override
  public void stop() {
    turretMotor.stopMotor();
  }

  @Override
  public void setBrakeMode() {
    turretMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void setCoastMode() {
    turretMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void setTurretPosition(double positionDegrees) {
    double rotations = positionDegrees / 360.0;
    // Clamp to safety bounds
    double safePosition =
        MathUtil.clamp(
            rotations, TurretConstants.MINROTATION + 0.05, TurretConstants.MAXROTATION - 0.05);
    turretMotor.setControl(motionMagicVoltage.withPosition(safePosition));
  }

  @Override
  public void setTurretVoltage(double voltage) {
    turretMotor.setControl(voltageOut.withOutput(voltage));
  }

  @Override
  public void setTurretSetPoint(double value) {
    setpointDegrees = value;
  }
}
