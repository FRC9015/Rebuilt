package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ShooterIOTalonFX implements ShooterIO {

  public final TalonFX flywheelMotorLeft;
  public final TalonFX flywheelMotorRight;
  public final TalonFX kickerMotor;

  public StatusSignal<Voltage> motorVolts, kickerVolts;
  public StatusSignal<Current> motorAmps, kickerAmps;
  public StatusSignal<AngularVelocity> motorRPM, kickerRPM;
  public StatusSignal<Angle> motorPosition;
  private LoggedNetworkNumber minPosition = new LoggedNetworkNumber("/Tuning/minPosition", 0.0);
  private LoggedNetworkNumber maxPosition = new LoggedNetworkNumber("/Tuning/maxPosition", 1.0);
  private MotionMagicVelocityVoltage flywheelMagicVelocityVoltage =
      new MotionMagicVelocityVoltage(00.);
  private MotionMagicVelocityVoltage kickerMagicVelocityVoltage =
      new MotionMagicVelocityVoltage(0.0);

  private double lastFlywheelSetpointSpeed = 0.0;

  public ShooterIOTalonFX(int flywheelID1, int flywheelID2, int kickerID) {
    flywheelMotorLeft = new TalonFX(flywheelID1);
    flywheelMotorRight = new TalonFX(flywheelID2);
    kickerMotor = new TalonFX(kickerID);

    // Configure motor
    TalonFXConfiguration flyWheelConfigLeft =
        new TalonFXConfiguration()
            .withSlot0(Constants.ShooterConstants.flyWheelSlotVelocityConfigs)
            .withMotionMagic(Constants.ShooterConstants.flyWheelMagicConfligs);

    flyWheelConfigLeft.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flyWheelConfigLeft.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    flyWheelConfigLeft.CurrentLimits.StatorCurrentLimit = 60;
    flyWheelConfigLeft.CurrentLimits.StatorCurrentLimitEnable = true;
    flyWheelConfigLeft.CurrentLimits.SupplyCurrentLimit = 60;
    flyWheelConfigLeft.CurrentLimits.SupplyCurrentLimitEnable = true;

    TalonFXConfiguration flyWheelConfigRight =
        new TalonFXConfiguration()
            .withSlot0(Constants.ShooterConstants.flyWheelSlotVelocityConfigs)
            .withMotionMagic(Constants.ShooterConstants.flyWheelMagicConfligs);

    flyWheelConfigRight.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    flyWheelConfigRight.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flyWheelConfigRight.CurrentLimits.StatorCurrentLimit = 60;
    flyWheelConfigRight.CurrentLimits.StatorCurrentLimitEnable = true;
    flyWheelConfigRight.CurrentLimits.SupplyCurrentLimit = 60;
    flyWheelConfigRight.CurrentLimits.SupplyCurrentLimitEnable = true;

    TalonFXConfiguration kickerConfig =
        new TalonFXConfiguration()
            .withSlot0(Constants.ShooterConstants.kickerSlotVelocityConfigs)
            .withFeedback(Constants.ShooterConstants.kickerFeedbackConfigs);

    kickerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    kickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    kickerConfig.CurrentLimits.StatorCurrentLimit = 30;
    kickerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    kickerConfig.CurrentLimits.SupplyCurrentLimit = 30;
    kickerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    flywheelMotorLeft.getConfigurator().apply(flyWheelConfigLeft);
    flywheelMotorRight.getConfigurator().apply(flyWheelConfigRight);
    kickerMotor.getConfigurator().apply(kickerConfig);

    motorVolts = flywheelMotorLeft.getMotorVoltage();
    motorAmps = flywheelMotorLeft.getStatorCurrent();
    motorRPM = flywheelMotorLeft.getVelocity();
    kickerVolts = kickerMotor.getMotorVoltage();
    kickerAmps = kickerMotor.getStatorCurrent();
    kickerRPM = kickerMotor.getVelocity();
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, motorVolts, motorAmps, motorRPM, kickerAmps, kickerRPM, kickerVolts);

    ParentDevice.optimizeBusUtilizationForAll(flywheelMotorLeft, flywheelMotorRight, kickerMotor);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        motorVolts, motorAmps, motorRPM, kickerAmps, kickerRPM, kickerVolts);
    inputs.flywheelAppliedVolts = motorVolts.getValueAsDouble();
    inputs.flywheelCurrentSpeed = flywheelMotorLeft.getVelocity().getValueAsDouble();
    inputs.flywheelRPM = motorRPM.getValueAsDouble();
    inputs.flywheelCurrentAmps = flywheelMotorLeft.getStatorCurrent().getValueAsDouble();
    inputs.flywheelTargetSpeed = lastFlywheelSetpointSpeed;
    inputs.kickerCurrentAmps = kickerAmps.getValueAsDouble();
    inputs.kickerAppliedVolts = kickerVolts.getValueAsDouble();
    inputs.kickerRPM = kickerRPM.getValueAsDouble();

    if (Math.abs(inputs.flywheelCurrentSpeed - inputs.flywheelTargetSpeed)
            < Constants.ShooterConstants.FLYWHEEL_RPM_TOLERANCE
        && inputs.flywheelTargetSpeed != 0.0) {
      inputs.flywheelAtSpeed = true;
    } else {
      inputs.flywheelAtSpeed = false;
    }
  }

  @Override
  public void stopFlywheels() {
    flywheelMotorLeft.stopMotor();
    flywheelMotorRight.stopMotor();
  }

  @Override
  public void stopKicker() {
    kickerMotor.stopMotor();
  }

  @Override
  public void stopShooter() {
    stopFlywheels();
    stopKicker();
  }

  @Override
  public void setBrakeMode(boolean enable) {
    flywheelMotorLeft.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    flywheelMotorRight.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  // Minimum Value of speedValue: -100 RPS
  // Maximum Value of speedValue: 100 RPS
  // Unit of output: RPS
  @Override
  public void setFlyWheelSpeed(double speed) {
    lastFlywheelSetpointSpeed = speed;
    flywheelMotorLeft.setControl(flywheelMagicVelocityVoltage.withVelocity(speed));
    flywheelMotorRight.setControl(flywheelMagicVelocityVoltage.withVelocity(speed));
  }

  @Override
  public void setKickerSpeed(double speed) {
    kickerMotor.set(speed);
  }

  @Override
  public double getFlyWheelSpeed() {
    double averageVelocity =
        (flywheelMotorLeft.getVelocity().getValueAsDouble()
                + flywheelMotorRight.getVelocity().getValueAsDouble())
            / 2;
    return averageVelocity;
  }
}
