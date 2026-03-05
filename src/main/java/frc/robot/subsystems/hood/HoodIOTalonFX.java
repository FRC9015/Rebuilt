package frc.robot.subsystems.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class HoodIOTalonFX implements HoodIO {
  public final TalonFX hoodMotor;

  public StatusSignal<Voltage> motorVolts;
  public StatusSignal<Current> motorAmps;
  public StatusSignal<AngularVelocity> motorRPM;
  public StatusSignal<Angle> motorPosition;
  private LoggedNetworkNumber minPosition = new LoggedNetworkNumber("/Tuning/minPosition", 0.0);
  private LoggedNetworkNumber maxPosition = new LoggedNetworkNumber("/Tuning/maxPosition", 1.0);
  private final MotionMagicVoltage hoodMagicVoltage =
      new MotionMagicVoltage(Constants.ShooterConstants.HOOD_MAX_POS);

  public HoodIOTalonFX(int hoodID) {

    hoodMotor = new TalonFX(hoodID);
    TalonFXConfiguration hoodConfig =
        new TalonFXConfiguration()
            .withSlot1(Constants.ShooterConstants.hoodSlotPositionConfigs)
            .withFeedback(Constants.ShooterConstants.hoodFeedbackConfigs)
            .withMotionMagic(Constants.ShooterConstants.hoodMagicConfigs);

    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = maxPosition.get();
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = minPosition.get();
    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    hoodMotor.getConfigurator().apply(hoodConfig);

    motorVolts = hoodMotor.getMotorVoltage();
    motorAmps = hoodMotor.getStatorCurrent();
    motorRPM = hoodMotor.getVelocity();
    motorPosition = hoodMotor.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, motorVolts, motorAmps, motorRPM, motorPosition);

    ParentDevice.optimizeBusUtilizationForAll(hoodMotor);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    BaseStatusSignal.refreshAll(motorVolts, motorAmps, motorRPM, motorPosition);
    inputs.hoodEncoderPosition = motorPosition.getValueAsDouble();
    inputs.hoodEncoderConnected = hoodMotor.getPosition().isAllGood();
    inputs.hoodAppliedVolts = motorVolts.getValueAsDouble();
    inputs.hoodCurrentAmps = motorAmps.getValueAsDouble();
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
        MathUtil.clamp(
            position,
            Constants.ShooterConstants.HOOD_MIN_POS,
            Constants.ShooterConstants.HOOD_MAX_POS);

    hoodMotor.setControl(hoodMagicVoltage.withPosition(clampedPosition).withSlot(0));
  }
}
