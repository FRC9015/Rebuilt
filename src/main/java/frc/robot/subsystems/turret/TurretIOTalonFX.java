package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.turretConstants;

public class TurretIOTalonFX implements TurretIO {

  private final TalonFX turretMotor;
  private final CANcoder encoder13;
  private final CANcoder encoder15;
  private final CANcoder driveGearEncoder;

  private final StatusSignal<Angle> encoder13PosSignal;
  private final StatusSignal<Angle> encoder15PosSignal;
  private final StatusSignal<Angle> driveEncoderPos;
  private final StatusSignal<Voltage> motorAppliedVoltsSignal;
  private final StatusSignal<Current> motorCurrentSignal;
  private final StatusSignal<Angle> motorPositionSignal;

  private final Debouncer encoderConnectedDebounce = new Debouncer(0.5);
  private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

  public TurretIOTalonFX(int motorID, int encoderId13, int encoderId15, int driveGearID) {
    turretMotor = new TalonFX(motorID);
    encoder13 = new CANcoder(encoderId13);
    encoder15 = new CANcoder(encoderId15);
    driveGearEncoder = new CANcoder(driveGearID);

    // Hardware-level soft limits provide physical protection even if code crashes
    TalonFXConfiguration motorConfig =
        new TalonFXConfiguration()
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(
                        turretConstants.MAXROTATION) // Stop at 1 rotations
                    .withReverseSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(
                        turretConstants.MINROTATION)) // Stop at -1 rotations
            .withMotionMagic(turretConstants.MOTION_MAGIC_CONFIGS)
            .withSlot0(turretConstants.SLOT0_CONFIGS)
            .withFeedback(turretConstants.FEEDBACK_CONFIGS)
            .withClosedLoopRamps(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.1));
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    turretMotor.getConfigurator().apply(motorConfig);

    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    CANcoderConfiguration driveGearConfig = new CANcoderConfiguration();

    driveGearConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

    encoder13.getConfigurator().apply(encoderConfig);
    encoder15.getConfigurator().apply(encoderConfig);
    driveGearEncoder.getConfigurator().apply(driveGearConfig);

    encoder13PosSignal = encoder13.getAbsolutePosition();
    encoder15PosSignal = encoder15.getAbsolutePosition();
    motorAppliedVoltsSignal = turretMotor.getMotorVoltage();
    motorCurrentSignal = turretMotor.getStatorCurrent();
    motorPositionSignal = turretMotor.getPosition();
    driveEncoderPos = driveGearEncoder.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        encoder13PosSignal,
        encoder15PosSignal,
        motorAppliedVoltsSignal,
        motorCurrentSignal,
        motorPositionSignal,
        driveEncoderPos);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        encoder13PosSignal,
        encoder15PosSignal,
        motorAppliedVoltsSignal,
        motorCurrentSignal,
        motorPositionSignal,
        driveEncoderPos);

    inputs.encoder13Connected =
        encoderConnectedDebounce.calculate(encoder13PosSignal.getStatus().isOK());
    inputs.encoder15Connected =
        encoderConnectedDebounce.calculate(encoder15PosSignal.getStatus().isOK());
    inputs.encoder13PositionRot = encoder13PosSignal.getValueAsDouble();
    inputs.encoder15PositionRot = encoder15PosSignal.getValueAsDouble();
    inputs.turretAppliedVolts = motorAppliedVoltsSignal.getValueAsDouble();
    inputs.turretCurrentAmps = motorCurrentSignal.getValueAsDouble();
    inputs.turretMotorPosition = motorPositionSignal.getValueAsDouble();
    inputs.driveEncoderPositionRot =
        driveEncoderPos.getValueAsDouble() * Constants.turretConstants.ENCODER_GEAR;
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
    // Convert degrees to rotations and clamp between -1.0 and 1.0
    double rotations = positionDegrees / 360.0;
    double safePosition = MathUtil.clamp(rotations, -1.0, 1.0);
    turretMotor.setControl(motionMagicVoltage.withPosition(safePosition));
  }
}
