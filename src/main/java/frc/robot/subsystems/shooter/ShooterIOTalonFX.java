package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ShooterIOTalonFX implements ShooterIO {
  public final TalonFX flywheelMotorLeft;
  public final TalonFX flywheelMotorRight;
  public final TalonFX hoodMotor;

  public StatusSignal motorVolts;
  public StatusSignal motorAmps;
  public StatusSignal motorRPM;
  public StatusSignal motorPosition;

  private LoggedNetworkNumber minPosition = new LoggedNetworkNumber("/Tuning/minPosition", 0.0);
  private LoggedNetworkNumber maxPosition = new LoggedNetworkNumber("/Tuning/maxPosition", 1.0);

  // Persistent control requests (Phoenix 6 pattern)
  private final MotionMagicVoltage hoodMagicVoltage = new MotionMagicVoltage(0.0).withSlot(0);
  private final VelocityVoltage leftVelocityRequest = new VelocityVoltage(0.0).withSlot(0);
  private final VelocityVoltage rightVelocityRequest = new VelocityVoltage(0.0).withSlot(0);
  private final VoltageOut leftVoltageOut = new VoltageOut(0.0);
  private final VoltageOut rightVoltageOut = new VoltageOut(0.0);

  public ShooterIOTalonFX(int flywheelID1, int flywheelID2, int hoodID) {
    flywheelMotorLeft = new TalonFX(flywheelID1);
    flywheelMotorRight = new TalonFX(flywheelID2);
    hoodMotor = new TalonFX(hoodID);

    TalonFXConfiguration flyWheelConfigLeft =
        new TalonFXConfiguration()
            .withSlot0(Constants.ShooterConstants.flyWheelSlotVelocityConfigs);
    TalonFXConfiguration flyWheelConfigRight =
        new TalonFXConfiguration()
            .withSlot0(Constants.ShooterConstants.flyWheelSlotVelocityConfigs);
    TalonFXConfiguration hoodConfig =
        new TalonFXConfiguration()
            .withSlot0(Constants.ShooterConstants.hoodSlotPositionConfigs)
            .withMotionMagic(Constants.ShooterConstants.HOOD_MAGIC_CONFIGS);

    flyWheelConfigLeft.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // flyWheelConfigLeft.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    flyWheelConfigLeft.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Match whatever inversion you proved correct in Tuner; this is your original setting.
    flyWheelConfigRight.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    flyWheelConfigRight.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    flywheelMotorLeft.getConfigurator().apply(flyWheelConfigLeft);
    flywheelMotorRight.getConfigurator().apply(flyWheelConfigRight);
    hoodMotor.getConfigurator().apply(hoodConfig);

    motorVolts = flywheelMotorLeft.getMotorVoltage();
    motorAmps = flywheelMotorLeft.getStatorCurrent();
    motorRPM = flywheelMotorLeft.getVelocity();
    motorPosition = hoodMotor.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, motorVolts, motorAmps, motorRPM, motorPosition);
    ParentDevice.optimizeBusUtilizationForAll(flywheelMotorLeft, flywheelMotorRight, hoodMotor);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(motorVolts, motorAmps, motorRPM, motorPosition);

    inputs.flywheelAppliedVolts = motorVolts.getValueAsDouble();
    inputs.hoodEncoderPosition = hoodMotor.getPosition().getValueAsDouble();
    inputs.flywheelCurrentSpeed = flywheelMotorLeft.getVelocity().getValueAsDouble();
    inputs.flywheelRPM = motorRPM.getValueAsDouble();
    inputs.flywheelAppliedVolts = motorVolts.getValueAsDouble();
    inputs.flywheelCurrentAmps = flywheelMotorLeft.getStatorCurrent().getValueAsDouble();
    inputs.hoodEncoderConnected = false;
  }

  @Override
  public void stopFlywheels() {
    flywheelMotorLeft.stopMotor();
    flywheelMotorRight.stopMotor();
  }

  @Override
  public void stopHood() {
    hoodMotor.stopMotor();
  }

  @Override
  public void stopShooter() {
    stopFlywheels();
    stopHood();
  }

  @Override
  public void setBrakeMode(boolean enable) {
    flywheelMotorLeft.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    flywheelMotorRight.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    hoodMotor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  // speed is in RPS
  @Override
  public void setFlyWheelSpeed(double speed) {
    // If Tuner showed "negative = outward", and you want outward as your API positive,
    // flip the sign here instead of in every caller:
    double talonSpeed = -speed;

    flywheelMotorLeft.setControl(
        leftVelocityRequest
            .withVelocity(-talonSpeed)
            .withAcceleration(Constants.ShooterConstants.FLYWHEEL_ACCELERATION)
            .withFeedForward(Constants.ShooterConstants.FEEDFORWARD_VOLTAGE));

    flywheelMotorRight.setControl(
        rightVelocityRequest
            .withVelocity(-talonSpeed)
            .withAcceleration(Constants.ShooterConstants.FLYWHEEL_ACCELERATION)
            .withFeedForward(Constants.ShooterConstants.FEEDFORWARD_VOLTAGE));
  }

  @Override
  public void setFlyWheelVoltage(double volts) {
    flywheelMotorLeft.setControl(leftVoltageOut.withOutput(volts));
    flywheelMotorRight.setControl(rightVoltageOut.withOutput(volts));
  }

  @Override
  public void setHoodPosition(double position) {
    hoodMotor.setControl(hoodMagicVoltage.withPosition(position));
    // hoodMotor.setPosition(position);
  }

  @Override
  public void setHoodZero() {
    hoodMotor.setControl(new VelocityVoltage(10.0));
  }

  @Override
  public void setHoodZeroReverse() {
    hoodMotor.setControl(new VelocityVoltage(-10.0));
  }

  public double getPosition() {
    return hoodMotor.getPosition().getValueAsDouble();
  }

  @Override
  public double getFlyWheelSpeed() {
    double averageVelocity =
        (flywheelMotorLeft.getVelocity().getValueAsDouble()
                + flywheelMotorRight.getVelocity().getValueAsDouble())
            / 2.0;
    return averageVelocity;
  }
}
