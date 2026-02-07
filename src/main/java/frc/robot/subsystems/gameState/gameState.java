package frc.robot.subsystems.gamestate;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;

import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class GameState extends SubsystemBase{
    /** gamestate: will be A for auto, T for transition, B for blue, R for red, and E for endgame */
    @AutoLogOutput
    private StateEnum state;
    @AutoLogOutput
    private boolean ishubactive;
    @AutoLogOutput
    private boolean willflash;
    @AutoLogOutput /**what team the robot is on, vscode sim returns PRACTICE, which will also be returned if connection to DS is instable */
    private StateEnum alliance;

    public GameState(){
        this.state = getGameState();
        this.ishubactive = getCanScore();
        this.alliance = findAlliance();
    }

    @Override
    public void periodic(){
        this.state = getGameState();
        this.ishubactive = getCanScore();
        this.willflash = getFlashy();
    }

    public StateEnum getstate(){
        return this.state;
    }

    public boolean getishubactive(){
        return this.ishubactive;
    }

    public boolean getwillflash(){
        return this.willflash;
    }

    public StateEnum getalliance(){
        return this.alliance;
    }

/**
 * finds the games current state
 * @return the game's state, A for auto, T for transition, B for blue, R for red, and E for endgame.
 */
    private StateEnum getGameState() throws NoSuchElementException{
        double time = -1;
        Boolean auto = DriverStation.isAutonomous();
        String gameData = DriverStation.getGameSpecificMessage();
        try {
            time = OptionalDouble.of(DriverStation.getMatchTime()).getAsDouble();
            if (auto == null) return StateEnum.ERROR;
            if (gameData == null) return StateEnum.ERROR;
        } catch (Exception noSuchElementException) {
            return StateEnum.ERROR;
        }
        if (auto){
            return StateEnum.AUTO;
        }
        if(time > 130){
        return StateEnum.TRANSITION;
        } else if (time > 30) {
        if ("B".equals(gameData)) {
            if (time > 105){
                return StateEnum.RED_TEAM;
            } else if (time>80) {
                return StateEnum.BLUE_TEAM;
            } else if (time > 55){
                return StateEnum.RED_TEAM;
            } else if(time > 30){
                return StateEnum.BLUE_TEAM;
            }
        }
            if("R".equals(gameData)){
            if (time > 105){
                return StateEnum.BLUE_TEAM;
            } else if (time>80) {
                return StateEnum.RED_TEAM;
            } else if (time > 55){
                return StateEnum.BLUE_TEAM;
            } else if(time > 30){
                return StateEnum.RED_TEAM;
        }
        }
    }
        if(time == -1){
            return StateEnum.PRACTICE; 
        }
        return StateEnum.ENDGAME;
    }
    /**
     * checks if the robot can score
     * @return true if tower enabled, false if tower disabled
     */
    private boolean getCanScore(){
        if ((this.state == StateEnum.BLUE_TEAM || this.state == StateEnum.RED_TEAM)&&
        this.state != this.alliance
        ) {
            return false;
        }else {
            return false;
        }
    }
    private boolean getFlashy(){
        double time = 0.0;
        time = DriverStation.getMatchTime();
        double epsilon = 0.21; 
        return 
        Math.abs(time - 133) < epsilon ||
        Math.abs(time - 108) < epsilon ||
        Math.abs(time - 83) < epsilon ||
        Math.abs(time - 58) < epsilon ||
        Math.abs(time - 33) < epsilon ||
        Math.abs(time - 3) < epsilon;
    }
    
    private StateEnum findAlliance(){
        Optional<Alliance> alli = DriverStation.getAlliance();
        Alliance allia = Alliance.Blue;
        try {
            allia = alli.orElseThrow();
        } catch (Exception NoSuchElementException) {
            return StateEnum.PRACTICE;
        }
        if (allia == Alliance.Blue){ return StateEnum.BLUE_TEAM;}
        else {return StateEnum.RED_TEAM;}
    }
    }