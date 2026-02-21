package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;

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
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;
import frc.robot.Constants.turretConstants;
import yams.units.EasyCRT;
import yams.units.EasyCRT.CRTStatus;
import yams.units.EasyCRTConfig;

public class TurretIOTalonFX implements TurretIO {

  private final TalonFX turretMotor;
  private final CANcoder encoder13;
  private final CANcoder encoder15;
  private final EasyCRT easyCRT;

  private final StatusSignal<Angle> encoder13PosSignal;
  private final StatusSignal<Angle> encoder15PosSignal;
  private final StatusSignal<Voltage> motorAppliedVoltsSignal;
  private final StatusSignal<Current> motorCurrentSignal;
  private final StatusSignal<Angle> motorPositionSignal;

  private final Debouncer encoderConnectedDebounce = new Debouncer(0.5);
  private final NeutralOut neutralOut = new NeutralOut();
  private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

  private Alert outofSyncAlert = new Alert("⚠️ CRT-Motor desync detected! Resyncing motor."
                       , AlertType.kWarning);
  private Alert outofBoundsAlert = new Alert("Turret out of bounds; no longer tracking", AlertType.kError);


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
                        turretConstants.MAXROTATION) // Stop at 1 rotations
                    .withReverseSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(
                        turretConstants.MINROTATION)) // Stop at -1 rotations
            .withMotionMagic(turretConstants.MOTION_MAGIC_CONFIGS)
            .withSlot0(turretConstants.SLOT0_CONFIGS)
            .withFeedback(turretConstants.FEEDBACK_CONFIGS)
            .withClosedLoopRamps(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.1));
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    turretMotor.getConfigurator().apply(motorConfig);

    CANcoderConfiguration encoderConfig13 = new CANcoderConfiguration();
    encoderConfig13.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoderConfig13.MagnetSensor.MagnetOffset = -0.352783;

    CANcoderConfiguration encoderConfig15 = new CANcoderConfiguration();
    encoderConfig15.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoderConfig15.MagnetSensor.MagnetOffset = -0.4035644;

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
            .withAbsoluteEncoder1Gearing(turretConstants.T_TEETH, turretConstants.E1_TEETH)
            .withAbsoluteEncoder2Gearing(turretConstants.T_TEETH, turretConstants.E2_TEETH)
            .withMechanismRange(
                Rotations.of(turretConstants.MINROTATION),
                Rotations.of(turretConstants.MAXROTATION))
            .withMatchTolerance(Rotations.of(turretConstants.CRT_TOLERANCE))
            .withAbsoluteEncoderOffsets(Rotations.of(0.0), Rotations.of(0.0))
            .withAbsoluteEncoderInversions(false, false);

    easyCRT = new EasyCRT(easyCRTConfig);

    // Log initialization result for debugging
    easyCRT
        .getAngleOptional()
        .ifPresentOrElse(
            angle -> {
              this.seedMotorPosition(angle.in(Rotations));
              System.out.println(
                  "✓ Turret CRT initialized at " + (angle.in(Rotations) * 360.0) + " degrees");
            },
            () -> {
              System.out.println("✗ CRT failed to resolve turret angle!");
              System.out.println("  Status: " + easyCRT.getLastStatus());
              System.out.println("  Enc13: " + encoder13.getAbsolutePosition().getValueAsDouble());
              System.out.println("  Enc15: " + encoder15.getAbsolutePosition().getValueAsDouble());
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

    // Resolve turret angle using CRT from two encoder readings
    easyCRT
        .getAngleOptional()
        .ifPresentOrElse(
            angle -> {
              inputs.turretResolvedValid = true;
              inputs.turretResolvedPosition = angle.in(Rotations);
              if (Math.abs(inputs.turretResolvedPosition - inputs.turretMotorPosition)
                  > Constants.turretConstants.SYNC_THRESHOLD) {
                    
                outofSyncAlert.set(Math.abs(inputs.turretResolvedPosition - inputs.turretMotorPosition)
                  > Constants.turretConstants.SYNC_THRESHOLD);

                this.seedMotorPosition(angle.in(Rotations));
              }
              // System.out.println("Turret CRT at " + (angle.in(Rotations) * 360.0) + " degrees");
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
  public void setTurretPosition(double positionRotations) {
    // Clamp to valid range before commanding motor
    double safePosition =
        MathUtil.clamp(
            positionRotations / 360, turretConstants.MINROTATION, turretConstants.MAXROTATION);
    turretMotor.setControl(motionMagicVoltage.withPosition(safePosition));
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
}
