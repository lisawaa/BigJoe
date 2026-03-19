package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class Intake extends Command {
    private final IntakeSubsystem intakeSub;
    private final double rpm;

    public Intake(IntakeSubsystem intakeSub, double rpm){
        this.intakeSub = intakeSub;
        this.rpm = rpm;
    }

    @Override 
    public void initialize(){
        intakeSub.setIntake(rpm);
    }

    @Override
    public void execute(){
    }

    @Override
    public void end(boolean interrupted){
        intakeSub.setIntake(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
