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
import frc.robot.Constants.turretConstants;

public class TurretIOTalonFX implements TurretIO {

  private final TalonFX turretMotor;
  private final CANcoder encoder13;
  private final CANcoder encoder15;

  private final StatusSignal<Angle> encoder13PosSignal;
  private final StatusSignal<Angle> encoder15PosSignal;
  private final StatusSignal<Voltage> motorAppliedVoltsSignal;
  private final StatusSignal<Current> motorCurrentSignal;
  private final StatusSignal<Angle> motorPositionSignal;

  private final Debouncer encoderConnectedDebounce = new Debouncer(0.5);
  private final NeutralOut neutralOut = new NeutralOut();
  private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

  public TurretIOTalonFX(int motorID, int encoderId13, int encoderId15) {
    turretMotor = new TalonFX(motorID);
    encoder13 = new CANcoder(encoderId13);
    encoder15 = new CANcoder(encoderId15);

    TalonFXConfiguration motorConfig =
        new TalonFXConfiguration()
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(turretConstants.maxRotation)
                    .withReverseSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(turretConstants.minRoation))
            .withMotionMagic(turretConstants.MOTION_MAGIC_CONFIGS)
            .withSlot0(turretConstants.SLOT0_CONFIGS)
            .withFeedback(turretConstants.FEEDBACK_CONFIGS)
            .withClosedLoopRamps(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.1));
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    turretMotor.getConfigurator().apply(motorConfig);

    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoder13.getConfigurator().apply(encoderConfig);
    encoder15.getConfigurator().apply(encoderConfig);

    encoder13PosSignal = encoder13.getAbsolutePosition();
    encoder15PosSignal = encoder15.getAbsolutePosition();
    motorAppliedVoltsSignal = turretMotor.getMotorVoltage();
    motorCurrentSignal = turretMotor.getStatorCurrent();
    motorPositionSignal = turretMotor.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        encoder13PosSignal,
        encoder15PosSignal,
        motorAppliedVoltsSignal,
        motorCurrentSignal,
        motorPositionSignal);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        encoder13PosSignal,
        encoder15PosSignal,
        motorAppliedVoltsSignal,
        motorCurrentSignal,
        motorPositionSignal);

    inputs.encoder13Connected =
        encoderConnectedDebounce.calculate(encoder13PosSignal.getStatus().isOK());
    inputs.encoder15Connected =
        encoderConnectedDebounce.calculate(encoder15PosSignal.getStatus().isOK());
    inputs.encoder13PositionRot = encoder13PosSignal.getValueAsDouble();
    inputs.encoder15PositionRot = encoder15PosSignal.getValueAsDouble();
    inputs.turretAppliedVolts = motorAppliedVoltsSignal.getValueAsDouble();
    inputs.turretCurrentAmps = motorCurrentSignal.getValueAsDouble();
    inputs.turretMotorPosition = motorPositionSignal.getValueAsDouble();

    Double resolvedPos =
        calculateTrueAngle(inputs.encoder13PositionRot, inputs.encoder15PositionRot);
    if (resolvedPos != null) {
      inputs.turretResolvedValid = true;
      inputs.turretResolvedPosition = resolvedPos;
    } else {
      inputs.turretResolvedValid = false;
    }
  }

  /**
   *
   *
   * <h3>Chinese Remainder Theorem (CRT) Resolver</h3>
   *
   * <p>This function determines the absolute position of the turret gear by comparing the readings
   * of two different-sized encoder gears. Because absolute single-turn encoders wrap around every
   * 360 degrees, a single encoder on a small gear does not know which "lap" the turret is on.
   * <b>The Logic:</b>
   *
   * <ol>
   *   <li>We normalize the encoder readings to a 0.0 - 1.0 range (representing 0-360 degrees).
   *   <li>An encoder on a small gear spins faster than the turret. Predicted Turret Rotation =
   *       (Laps + Reading) * (Encoder_Teeth / Turret_Teeth).
   *   <li>Because we don't know the "Laps" (n and k), we test every mathematically possible lap
   *       count for both encoders.
   *   <li>For each combination of laps, we calculate where Encoder 1 thinks the turret is, and
   *       where Encoder 2 thinks the turret is.
   *   <li>When both predictions align within {@link turretConstants#CRT_TOLERANCE}, we have found
   *       the unique absolute position of the turret.
   * </ol>
   *
   * @param raw13 The 0-1 absolute rotation reading from the 13-tooth gear encoder.
   * @param raw15 The 0-1 absolute rotation reading from the 15-tooth gear encoder.
   * @return The absolute rotation of the turret (e.g., 1.5 rotations), or null if no match is
   *     found.
   */
  private Double calculateTrueAngle(double raw13, double raw15) {
    double val13 = MathUtil.inputModulus(raw13, 0, 1.0);
    double val15 = MathUtil.inputModulus(raw15, 0, 1.0);

    for (int n = 0; n < turretConstants.E1_SEARCH_LIMIT; n++) {
      double attemptA = (n + val13) * (turretConstants.e1_teeth / turretConstants.t_teeth);

      for (int k = 0; k < turretConstants.E2_SEARCH_LIMIT; k++) {
        double attemptB = (k + val15) * (turretConstants.e2_teeth / turretConstants.t_teeth);

        if (Math.abs(attemptA - attemptB) < turretConstants.CRT_TOLERANCE) {
          return attemptA;
        }
      }
    }
    return null;
  }

  @Override
  public void setturretPosition(double value) {
    turretMotor.setControl(motionMagicVoltage.withPosition(value));
  }

  @Override
  public void seedMotorPosition(double positionRotations) {
    turretMotor.setPosition(positionRotations);
  }

  @Override
  public void stop() {
    turretMotor.setControl(neutralOut);
  }

  @Override
  public void setBrakeMode() {
    turretMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void setCoastMode() {
    turretMotor.setNeutralMode(NeutralModeValue.Coast);
  }
}
