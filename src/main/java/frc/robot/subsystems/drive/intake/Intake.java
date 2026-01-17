package frc.robot.subsystems.drive.intake;

import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.generated.TunerConstants;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.PhoenixUtil;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    

    public Command extendIntake(){
        return this.runOnce(this::extend);
    }
    /**
     * intakes fuel, plz use toggle
     */
    public Command intakeFuel(){
        return this.run(this::setIntakeMotorSpeeds);
    }
    private void setIntakeMotorSpeeds(){

    }
    private void extend(){

    }
}
