package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class FeedToShoot extends Command {
    private final ShooterSubsystem shooterSub;
    
    public FeedToShoot(ShooterSubsystem shooterSub){
        this.shooterSub = shooterSub;
    }

    @Override 
    public void initialize(){
        shooterSub.setFeeder(0.9);
        shooterSub.setHopper(0.2);
    }

    @Override
    public void execute(){}

    @Override
    public void end(boolean interrupted){
        shooterSub.setFeeder(0);
        shooterSub.setHopper(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
