package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
  private final MotionMagicExpoVoltage pivotMagicVoltage = new MotionMagicExpoVoltage(0.0);

  public PivotIOTalonFX(int pivotIDLeft, int encoderID) {
    pivotMotorLeft = new TalonFX(pivotIDLeft);
    pivotEncoder = new CANcoder(encoderID);

    pivotMotorLeft.getConfigurator().apply(IntakeConstants.pivotConfigLeft);
    pivotLeftVolts = pivotMotorLeft.getMotorVoltage();
    pivotLeftAmps = pivotMotorLeft.getStatorCurrent();
    pivotLeftVelocity = pivotMotorLeft.getVelocity();
    pivotPosition = pivotEncoder.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        pivotLeftVolts,
        pivotLeftAmps,
        pivotLeftVelocity,
        pivotPosition);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        pivotLeftVolts,
        pivotLeftAmps,
        pivotLeftVelocity,
        pivotPosition);
    inputs.pivotLeftAppliedVolts = pivotLeftVolts.getValueAsDouble();
    inputs.pivotLeftCurrentSpeed = pivotLeftVelocity.getValueAsDouble();
    inputs.pivotLeftCurrentAmps = pivotLeftAmps.getValueAsDouble();
    inputs.pivotPosition = pivotPosition.getValueAsDouble();
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
        MathUtil.clamp(position, IntakeConstants.INTAKE_MIN_POS, IntakeConstants.INTAKE_MAX_POS);
    pivotMotorLeft.setControl(pivotMagicVoltage.withPosition(clampedPosition));
  }

  @Override
  public void setVolts(double volts) {
    pivotMotorLeft.setVoltage(volts);
  }

  @Override
  public void setPivotPosition(PivotPositions position) {
    pivotMotorLeft.setControl(pivotMagicVoltage.withPosition(position.getPivotPosition()));
  }
}
