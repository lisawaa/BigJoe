package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Configs;
import frc.robot.Constants.IDs;
import frc.robot.components.PIDMotor;
import frc.robot.components.PIDMotorIOSparkMax;

public class ClimbSubsystem extends SubsystemBase{
    
    private final PIDMotor innerLeft; 
    private final PIDMotor innerRight;
    private final DigitalInput limitSwitch;

    public ClimbSubsystem() {
        innerLeft = new PIDMotor(new PIDMotorIOSparkMax(IDs.ClimbConstants.CLIMB_LEFT_ID, Configs.Climb.LEFT_CONFIG));
        innerRight = new PIDMotor(new PIDMotorIOSparkMax(IDs.ClimbConstants.CLIMB_RIGHT_ID, Configs.Climb.RIGHT_CONFIG));
        limitSwitch = new DigitalInput(0);
        resetEncoders();
    }
    
    public void move(double speed) {
        innerLeft.set(speed);
        innerRight.set(speed);
    }

    public void raise() {
        while(innerLeft.getEncoder() > -317)
            move(-0.8);
        move(0);
    }

    public void retract() {
        while(innerLeft.getEncoder() < 0)
            move(0.8);
        move(0);
    }

    public void resetEncoders() {
        innerLeft.resetEncoder();
        innerRight.resetEncoder();
    }

    public void stop() {
        innerLeft.stopMotors();
        innerRight.stopMotors();
    }

    public double getEncoder() {
        return innerLeft.getEncoder();
    }

    @Override
    public void periodic () {
        if(!limitSwitch.get()) {
            resetEncoders();
        }
        Logger.recordOutput("Climb/Left", innerLeft.getEncoder());
        Logger.recordOutput("Climb/Right", innerRight.getEncoder());
        Logger.recordOutput("Climb/Switch", !limitSwitch.get());
    }
}
