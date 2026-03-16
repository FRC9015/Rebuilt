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

  public final TalonFX rollerMotor;

  public StatusSignal<Voltage> rollerVolts;
  public StatusSignal<Current> rollerAmps;
  public StatusSignal<AngularVelocity> rollerSpeed;

  private final MotionMagicVelocityVoltage intakeVelocityVoltage =
      new MotionMagicVelocityVoltage(0.0);
  private final VoltageOut intakeVoltageOut = new VoltageOut(0.0);

  public RollerIOTalonFX(int rollerID) {
    rollerMotor = new TalonFX(rollerID);

    rollerMotor.getConfigurator().apply(IntakeConstants.rollerConfigLeft);
    rollerVolts = rollerMotor.getMotorVoltage();
    rollerAmps = rollerMotor.getStatorCurrent();
    rollerSpeed = rollerMotor.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, rollerVolts, rollerAmps, rollerSpeed);

    ParentDevice.optimizeBusUtilizationForAll(rollerMotor);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {

    BaseStatusSignal.refreshAll(rollerVolts, rollerAmps, rollerSpeed);

    inputs.rollerLeftAppliedVolts = rollerVolts.getValueAsDouble();
    inputs.rollerLeftCurrentSpeed = rollerSpeed.getValueAsDouble();
    inputs.rollerLeftCurentAmps = rollerAmps.getValueAsDouble();
  }

  @Override
  public void stop() {
    rollerMotor.stopMotor();
  }

  @Override
  public void setBrakeMode(boolean enable) {
    rollerMotor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  // Minimum Value of speedValue: -512.0
  // Maximum Value of speedValkue: 511.998046875
  // Unit of output: RPS
  @Override
  public void setRollerSpeed(double speed) {
    rollerMotor.setControl(
        intakeVelocityVoltage.withVelocity(
            MathUtil.clamp(
                speed, IntakeConstants.INTAKE_MIN_SPEED, IntakeConstants.INTAKE_MAX_SPEED)));
  }

  @Override
  public void setRollerVolts(double voltage) {
    rollerMotor.setControl(intakeVoltageOut.withOutput(voltage));
  }
}
