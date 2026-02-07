package frc.robot.subsystems.gamestate;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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
import frc.robot.generated.TunerConstants;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.PhoenixUtil;

import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class GameState extends SubsystemBase{
    /** gamestate: will be A for auto, T for transition, B for blue, R for red, and E for endgame */
    private StateEnum state;
    private boolean ishubactive;
    private boolean willflash;
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
    @AutoLogOutput
    private StateEnum getGameState (){
        double time = DriverStation.getMatchTime();
        if (DriverStation.isAutonomous()){
            return StateEnum.AUTO;
        }
        if(time > 130){
        return StateEnum.TRANSITION;
        } else if (time > 30) {
        String gameData = DriverStation.getGameSpecificMessage();
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
        if (this.state == StateEnum.AUTO|| 
        this.state == StateEnum.TRANSITION|| 
        this.state == StateEnum.ENDGAME|| 
        this.state == StateEnum.PRACTICE||
        this.alliance == StateEnum.PRACTICE||
        this.state == this.alliance
        ) {
            return true;
        }else {
            return false;
        }
    }
    private boolean getFlashy(){
        double time = DriverStation.getMatchTime();
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
