package frc.robot.subsystems.drive.intake;

import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.generated.TunerConstants;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.PhoenixUtil;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;


public class Intake extends SubsystemBase {
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private final IntakeIO io;
    private final Alert EncoderDisconnectedAlert;
    private final TalonFX UpperMotor;
    private final TalonFX ExtendMotor;



  public Intake(IntakeIO io) {
    this.io = io;
    EncoderDisconnectedAlert = new Alert("Intake encoder disconnected!", AlertType.kError);
    UpperMotor = new TalonFX(Constants.motorIDConstants.UPPER_INTAKE_MOTOR_ID, TunerConstants.swerveDrivetrainConstants.CANBusName);
    ExtendMotor = new TalonFX(Constants.motorIDConstants.EXTEND_INTAKE_MOTOR_ID, TunerConstants.swerveDrivetrainConstants.CANBusName);
  }    

    public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    }

    /**
     * puts intake in active position
     */
    public Command extendIntake(){
    return this.startRun(
    this::extend, 
    this::setIntakeMotorSpeeds);    }
    
    public Command retractIntake(){
        return new SequentialCommandGroup(
            this.runOnce(this::stopIntakeMotors),
            this.runOnce(this::retract));
    }


    
    private void setIntakeMotorSpeeds(){
        UpperMotor.set(0.8);
        Logger.recordOutput("Intake/Set", true);
    }
    
    private void stopIntakeMotors(){
        UpperMotor.stopMotor();
        Logger.recordOutput("Intake/Stopped", true);
    }

    private void extend(){
        //write when i figure out what we're doing
        Logger.recordOutput("Intake/Extended", true);
    }

    private void retract(){
        
        Logger.recordOutput("Intake/Retracted", true);
    }
}
