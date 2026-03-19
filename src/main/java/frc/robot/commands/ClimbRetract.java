package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbRetract extends Command{
    private final ClimbSubsystem climbSub;

    public ClimbRetract(ClimbSubsystem climbSub){
        this.climbSub = climbSub;
        addRequirements(climbSub);
    }

    @Override 
    public void initialize(){
        climbSub.move(0.8);
    }

    @Override
    public void execute(){
        if(climbSub.getEncoder() > 0)
            climbSub.move(0);
    }

    @Override
    public void end(boolean interrupted){
        climbSub.move(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
