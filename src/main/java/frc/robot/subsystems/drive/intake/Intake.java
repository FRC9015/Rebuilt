package frc.robot.subsystems.drive.intake;

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
