package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.shared.OperatorConstants.RobotMode;

public class Superstructure {
    private final KitbotIntakeSubsystem intakeSubsystem;
    private final KitbotShootingSubsystem shootingSubsystem;
    private boolean hubActive;
    private RobotMode currentMode;
    private RobotMode targetMode;

    public Superstructure(KitbotIntakeSubsystem intake, KitbotShootingSubsystem shooting) {
        intakeSubsystem = intake;
        shootingSubsystem = shooting;
        hubActive = false; 
        currentMode = RobotMode.IDLE;
        targetMode = RobotMode.IDLE;
    }

    public void setTargetMode(RobotMode mode) {
        targetMode = mode;
        System.out.println("Target mode set to: " + targetMode);
    }

    public RobotMode getCurrentMode() {
        return currentMode;
    }

    public RobotMode getTargetMode() {
        return targetMode;
    }

    public boolean atTargetMode(){
        return currentMode == targetMode;
    }

    public void updateHubState(boolean autoWon){ //could do this in a cleaner way
        double matchTime = DriverStation.getMatchTime();
        if(DriverStation.isAutonomous())
            hubActive = true;
        else if(matchTime >= 130) //Transition shift
            hubActive = true;
        else if(autoWon && matchTime >= 105) //shift 1 won auto
            hubActive = false;
        else if(!autoWon && matchTime >= 105)//shift 1 lost auto
            hubActive = true;
        else if(autoWon && matchTime >= 80)//shift 2 won auto
            hubActive = true;
        else if(!autoWon && matchTime >= 80)//shift 2 lost auto
            hubActive = false;
        else if(autoWon && matchTime >= 55)//shift 3 won auto
            hubActive = false;
        else if(!autoWon && matchTime >= 55)//shift 3 lost auto
            hubActive = true;
        else //last 30 seconds
            hubActive = true;
    }

    public boolean isHubActive(){
        return hubActive;    
    }

    public void periodic(){
        //logging needed
        switch(currentMode){
            case IDLE:
                handleIdleMode();
                break; 
            case SHOOTING:
                handleShootingMode();
                break;
            case INTAKING_PREP:
                handleIntakingPrepMode();
                break;
            case INTAKING:
                handleIntakingMode();
                break;
            case SHOOTING_PREP:
                handleShootingPrepMode();
                break;
        }
    }

    public void handleIdleMode(){
        intakeSubsystem.stopIntake();
        shootingSubsystem.stopMotors();
        currentMode = RobotMode.IDLE;
    }

    public void handleShootingMode(){
        shootingSubsystem.shooting();
        intakeSubsystem.stopIntake();
        currentMode = RobotMode.SHOOTING;
    }

    public void handleIntakingMode(){
        intakeSubsystem.Intake();
        shootingSubsystem.stopMotors();
        currentMode = RobotMode.INTAKING;
    }

    public void handleShootingPrepMode(){
        intakeSubsystem.stopIntake();
        currentMode = RobotMode.SHOOTING_PREP; 
    }

    public void handleIntakingPrepMode(){
        shootingSubsystem.stopMotors();
        currentMode = RobotMode.INTAKING_PREP;
    }

}