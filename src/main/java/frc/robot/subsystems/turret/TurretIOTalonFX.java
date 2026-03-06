package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
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
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import org.littletonrobotics.junction.Logger;
import yams.units.EasyCRT;
import yams.units.EasyCRT.CRTStatus;
import yams.units.EasyCRTConfig;

/**
 * TurretIOTalonFX for actually running the turret and updating the turretIO file using a TalonFX
 * motor.
 */
public class TurretIOTalonFX implements TurretIO {

  private final TalonFX turretMotor;
  private final CANcoder encoder13;
  private final CANcoder encoder15;
  private final EasyCRT easyCRT;
  private double setpointDegrees = 0.0;

  private final StatusSignal<Angle> encoder13PosSignal;
  private final StatusSignal<Angle> encoder15PosSignal;
  private final StatusSignal<Voltage> motorAppliedVoltsSignal;
  private final StatusSignal<Current> motorCurrentSignal;
  private final StatusSignal<Angle> motorPositionSignal;

  private final Debouncer encoderConnectedDebounce = new Debouncer(0.5);
  private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);
  private final VoltageOut voltageOut = new VoltageOut(0.0);

  private Alert outofSyncAlert =
      new Alert("CRT-Motor desync detected! Resyncing motor.", AlertType.kWarning);
  private Alert outofBoundsAlert =
      new Alert("Turret out of bounds; no longer tracking", AlertType.kError);

  public TurretIOTalonFX(int motorID, int encoderId13, int encoderId15) {
    turretMotor = new TalonFX(motorID);
    encoder13 = new CANcoder(encoderId13);
    encoder15 = new CANcoder(encoderId15);

    // Hardware-level soft limits provide physical protection even if code crashes
    TalonFXConfiguration motorConfig =
        new TalonFXConfiguration()
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(
                        TurretConstants.MAXROTATION) // Stop at 1 rotations
                    .withReverseSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(
                        TurretConstants.MINROTATION)) // Stop at -1 rotations
            .withMotionMagic(TurretConstants.MOTION_MAGIC_CONFIGS)
            .withSlot0(TurretConstants.SLOT0_CONFIGS)
            .withFeedback(TurretConstants.FEEDBACK_CONFIGS)
            .withClosedLoopRamps(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.1));
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    turretMotor.getConfigurator().apply(motorConfig);

    CANcoderConfiguration encoderConfig13 = new CANcoderConfiguration();
    encoderConfig13.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoderConfig13.MagnetSensor.MagnetOffset = TurretConstants.ENCODER13_MAGNET_OFFSET;

    CANcoderConfiguration encoderConfig15 = new CANcoderConfiguration();
    encoderConfig15.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoderConfig15.MagnetSensor.MagnetOffset = TurretConstants.ENCODER15_MAGNET_OFFSET;

    encoder13.getConfigurator().apply(encoderConfig13);
    encoder15.getConfigurator().apply(encoderConfig15);

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

    // Configure EasyCRT to resolve turret angle from two encoders using Chinese Remainder Theorem
    EasyCRTConfig easyCRTConfig =
        new EasyCRTConfig(
                () -> encoder13.getAbsolutePosition().getValue(),
                () -> encoder15.getAbsolutePosition().getValue())
            .withAbsoluteEncoder1Gearing(TurretConstants.T_TEETH, TurretConstants.E1_TEETH)
            .withAbsoluteEncoder2Gearing(TurretConstants.T_TEETH, TurretConstants.E2_TEETH)
            .withMechanismRange(
                Rotations.of(TurretConstants.MINROTATION),
                Rotations.of(TurretConstants.MAXROTATION))
            .withMatchTolerance(Rotations.of(TurretConstants.CRT_TOLERANCE))
            .withAbsoluteEncoderOffsets(Rotations.of(0.0), Rotations.of(0.0))
            .withAbsoluteEncoderInversions(false, false);

    easyCRT = new EasyCRT(easyCRTConfig);

    // Log initialization result for debugging
    easyCRT
        .getAngleOptional()
        .ifPresentOrElse(
            angle -> {
              this.seedMotorPosition(angle.in(Rotations));
              Logger.recordOutput(
                  "Turret/InitStatus",
                  "Turret CRT initialized at " + (angle.in(Rotations) * 360.0) + " degrees");
            },
            () -> {
              Logger.recordOutput("Turret/InitStatus", "CRT failed to resolve turret angle!");
              Logger.recordOutput("Turret/InitStatuslast", "  Status: " + easyCRT.getLastStatus());
              Logger.recordOutput(
                  "Turret/InitStatus13",
                  "  Enc13: " + encoder13.getAbsolutePosition().getValueAsDouble());
              Logger.recordOutput(
                  "Turret/InitStatus15",
                  "  Enc15: " + encoder15.getAbsolutePosition().getValueAsDouble());
            });
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
    inputs.turretSetpoint = setpointDegrees;
    inputs.turretError = (Math.abs(inputs.turretResolvedPosition - inputs.turretSetpoint)) * 360;

    // Resolve turret angle using CRT from two encoder readings
    easyCRT
        .getAngleOptional()
        .ifPresentOrElse(
            angle -> {
              inputs.turretResolvedValid = true;
              inputs.turretResolvedPosition = angle.in(Rotations);
              inputs.turretResolvedPositionDegrees = angle.in(Rotations) * 360.0;
              double motorPositionWrapped =
                  MathUtil.inputModulus(
                      inputs.turretMotorPosition,
                      TurretConstants.MINROTATION,
                      TurretConstants.MAXROTATION);

              if (Math.abs(inputs.turretResolvedPosition - motorPositionWrapped)
                  > Constants.TurretConstants.SYNC_THRESHOLD) {

                outofSyncAlert.set(
                    Math.abs(inputs.turretResolvedPosition - motorPositionWrapped)
                        > Constants.TurretConstants.SYNC_THRESHOLD);

                inputs.turretResolvedPosition = motorPositionWrapped;
              }
            },
            () -> {
              inputs.turretResolvedValid = false;
              outofBoundsAlert.set(!inputs.turretResolvedValid);
            });
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
    // Convert degrees to rotations and clamp between -1.0 and 1.0
    double rotations = positionDegrees / 360.0;
    double safePosition = MathUtil.clamp(rotations, -0.7, 0.7);
    turretMotor.setControl(motionMagicVoltage.withPosition(safePosition));
    setpointDegrees = safePosition;
  }

  @Override
  public void setTurretVoltage(double voltage) {
    turretMotor.setControl(voltageOut.withOutput(voltage));
  }

  public CRTStatus getLastCRTStatus() {
    return easyCRT.getLastStatus();
  }

  public double getLastCRTErrorRotations() {
    return easyCRT.getLastErrorRotations();
  }

  public int getLastCRTIterations() {
    return easyCRT.getLastIterations();
  }

  @Override
  public void setTurretSetPoint(double value) {
    setpointDegrees = value;
  }
}
