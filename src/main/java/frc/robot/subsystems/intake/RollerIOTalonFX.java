package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.IntakeConstants;

public class RollerIOTalonFX implements RollerIO {

  public final TalonFX rollerMotorLeft;

  public StatusSignal<Voltage> rollerLeftVolts;
  public StatusSignal<Current> rollerLeftAmps;
  public StatusSignal<AngularVelocity> rollerLeftspeed;

  private final MotionMagicVelocityVoltage intakeVelocityVoltage =
      new MotionMagicVelocityVoltage(0.0);
  private final VoltageOut intakeVoltageOut = new VoltageOut(0.0);

  public RollerIOTalonFX(int rollerIDLeft) {
    rollerMotorLeft = new TalonFX(rollerIDLeft);

    rollerMotorLeft.getConfigurator().apply(IntakeConstants.rollerConfigLeft);
    rollerLeftVolts = rollerMotorLeft.getMotorVoltage();
    rollerLeftAmps = rollerMotorLeft.getStatorCurrent();
    rollerLeftspeed = rollerMotorLeft.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, rollerLeftVolts, rollerLeftAmps, rollerLeftspeed);

    ParentDevice.optimizeBusUtilizationForAll(rollerMotorLeft);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {

    BaseStatusSignal.refreshAll(rollerLeftVolts, rollerLeftAmps, rollerLeftspeed);

    inputs.rollerLeftAppliedVolts = rollerLeftVolts.getValueAsDouble();
    inputs.rollerLeftCurrentSpeed = rollerLeftspeed.getValueAsDouble();
    inputs.rollerLeftCurentAmps = rollerLeftAmps.getValueAsDouble();
  }

  @Override
  public void stop() {
    rollerMotorLeft.stopMotor();
  }

  @Override
  public void setBrakeMode(boolean enable) {
    rollerMotorLeft.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  // Minimum Value of speedValue: -512.0
  // Maximum Value of speedValkue: 511.998046875
  // Unit of output: RPS
  @Override
  public void setRollerSpeed(double speed) {
    rollerMotorLeft.setControl(
        intakeVelocityVoltage.withVelocity(
            MathUtil.clamp(
                speed, IntakeConstants.INTAKE_MIN_SPEED, IntakeConstants.INTAKE_MAX_SPEED)));
  }

  @Override
  public void setRollerVolts(double voltage) {
    rollerMotorLeft.setControl(intakeVoltageOut.withOutput(voltage));
  }
}
