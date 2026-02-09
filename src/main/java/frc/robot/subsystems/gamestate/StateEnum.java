package frc.robot.subsystems.gamestate;

public enum StateEnum {
    /**
     * for when the match is in auto
     */
    AUTO,
    /**
     * for when the match is in transition period
     */
    TRANSITION,
    /**
     * for when the blue teams hub is active
     */
    BLUE_TEAM,
    /**
     * for when the red teams hub is active
     */
    RED_TEAM,
    /**
     * for when the match is in endgame
     */
    ENDGAME,
    /**
     * for when robot is being practiced or tested outside of a certain state; to use set time to -1 (default in ds gui)
     */
    PRACTICE,
    /**
     * when the robot is not set to anything in particular or the winner of auto is unknown but needed information
     */
    UNKNOWN
}
