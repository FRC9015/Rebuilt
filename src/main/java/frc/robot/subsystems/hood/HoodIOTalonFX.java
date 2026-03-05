package frc.robot.subsystems.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class HoodIOTalonFX implements HoodIO {
  public final TalonFX hoodMotor;
  public final CANcoder hoodEncoder;

  public StatusSignal<Voltage> motorVolts;
  public StatusSignal<Current> motorAmps;
  public StatusSignal<AngularVelocity> motorRPM;
  public StatusSignal<Angle> motorPosition, encoderPosition;
  private LoggedNetworkNumber minPosition = new LoggedNetworkNumber("/Tuning/minPosition", 0.0);
  private LoggedNetworkNumber maxPosition = new LoggedNetworkNumber("/Tuning/maxPosition", 1.0);
  private final MotionMagicVoltage hoodMagicVoltage =
      new MotionMagicVoltage(ShooterConstants.HOOD_MAX_POS);
  private double target;

  public HoodIOTalonFX(int hoodID, int encoderID) {

    hoodMotor = new TalonFX(hoodID);
    hoodEncoder = new CANcoder(encoderID);

    TalonFXConfiguration hoodConfig =
        new TalonFXConfiguration()
            .withSlot1(ShooterConstants.hoodSlotPositionConfigs)
            .withFeedback(ShooterConstants.hoodFeedbackConfigs)
            .withMotionMagic(ShooterConstants.hoodMagicConfigs);

    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = maxPosition.get();
    // hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = minPosition.get();
    // hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    CANcoderConfiguration hoodEncoderConfig = new CANcoderConfiguration();
    hoodEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    hoodEncoderConfig.MagnetSensor.MagnetOffset = ShooterConstants.HOOD_ENCODER_OFFSET;
    hoodMotor.getConfigurator().apply(hoodConfig);
    hoodEncoder.getConfigurator().apply(hoodEncoderConfig);

    motorVolts = hoodMotor.getMotorVoltage();
    motorAmps = hoodMotor.getStatorCurrent();
    motorRPM = hoodMotor.getVelocity();
    motorPosition = hoodMotor.getPosition();
    encoderPosition = hoodEncoder.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, motorVolts, motorAmps, motorRPM, motorPosition);

    ParentDevice.optimizeBusUtilizationForAll(hoodMotor);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    BaseStatusSignal.refreshAll(motorVolts, motorAmps, motorRPM, motorPosition, encoderPosition);
    inputs.hoodEncoderPosition = encoderPosition.getValueAsDouble();
    inputs.hoodMotorPosition = motorPosition.getValueAsDouble();
    inputs.hoodEncoderConnected = hoodMotor.getPosition().isAllGood();
    inputs.hoodAppliedVolts = motorVolts.getValueAsDouble();
    inputs.hoodCurrentAmps = motorAmps.getValueAsDouble();
    inputs.hoodTargetPosition = target;
  }

  @Override
  public void stopHood() {
    hoodMotor.stopMotor();
  }

  @Override
  public void setBrakeMode(boolean enable) {
    hoodMotor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setHoodPosition(double position) {
    final double clampedPosition =
        MathUtil.clamp(position, ShooterConstants.HOOD_MIN_POS, ShooterConstants.HOOD_MAX_POS);

    target = clampedPosition;

    hoodMotor.setControl(hoodMagicVoltage.withPosition(clampedPosition).withSlot(1));
  }
}
