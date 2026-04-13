package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.IntakeConstants;

public class PivotIOTalonFX implements PivotIO {
  public final TalonFX pivotMotorLeft;
  public final CANcoder pivotEncoder;
  public StatusSignal<Voltage> pivotLeftVolts;
  public StatusSignal<Current> pivotLeftAmps;
  public StatusSignal<AngularVelocity> pivotLeftVelocity;
  public StatusSignal<Angle> pivotPosition;
  public StatusSignal<Angle> pivotPosition2;
  private final MotionMagicExpoVoltage pivotMagicVoltage = new MotionMagicExpoVoltage(0.0);

  public double localSetpoint = 0.0;
  public boolean isDeployed = false;

  public PivotIOTalonFX(int pivotIDLeft, int encoderID) {
    pivotMotorLeft = new TalonFX(pivotIDLeft);
    pivotEncoder = new CANcoder(encoderID);

    pivotMotorLeft.getConfigurator().apply(IntakeConstants.pivotConfigLeft);

    CANcoderConfiguration encoderConifg = new CANcoderConfiguration();

    encoderConifg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    pivotEncoder.getConfigurator().apply(encoderConifg);
    pivotLeftVolts = pivotMotorLeft.getMotorVoltage();
    pivotLeftAmps = pivotMotorLeft.getStatorCurrent();
    pivotLeftVelocity = pivotMotorLeft.getVelocity();
    pivotPosition = pivotEncoder.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, pivotLeftVolts, pivotLeftAmps, pivotLeftVelocity, pivotPosition);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    BaseStatusSignal.refreshAll(pivotLeftVolts, pivotLeftAmps, pivotLeftVelocity, pivotPosition);

    pivotMotorLeft.setPosition(pivotPosition.getValueAsDouble());

    inputs.pivotLeftAppliedVolts = pivotLeftVolts.getValueAsDouble();
    inputs.pivotLeftCurrentSpeed = pivotLeftVelocity.getValueAsDouble();
    inputs.pivotLeftCurrentAmps = pivotLeftAmps.getValueAsDouble();
    inputs.pivotPosition = pivotPosition.getValueAsDouble();

    inputs.setpoint = localSetpoint;
    inputs.setpointError = inputs.setpoint - inputs.pivotPosition2;
    inputs.isDeployed = isDeployed;
  }

  @Override
  public void stop() {
    pivotMotorLeft.stopMotor();
  }

  @Override
  public void setBrakeMode(boolean enable) {
    pivotMotorLeft.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setPivotPosition(double position) {
    final double clampedPosition =
        MathUtil.clamp(position, IntakeConstants.PIVOT_MIN_POS, IntakeConstants.PIVOT_MAX_POS);

    localSetpoint = clampedPosition;

    if (position == PivotPositions.DEPLOYED.getPivotPosition()) {
      isDeployed = true;
    } else {
      isDeployed = false;
    }

    pivotMotorLeft.setControl(pivotMagicVoltage.withPosition(clampedPosition));
  }

  @Override
  public void setVolts(double volts) {
    pivotMotorLeft.setVoltage(volts);
  }

  @Override
  public void seedPivotPosition(double position) {
    pivotMotorLeft.setPosition(position);
  }

  @Override
  public void setPivotPosition(PivotPositions position) {
    pivotMotorLeft.setControl(pivotMagicVoltage.withPosition(position.getPivotPosition()));
  }
}
