package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.Configs.Shooter;
import frc.robot.Constants.IDs.ShooterConstants;
import frc.robot.components.LinearInterpolator;
import frc.robot.components.PIDMotor;
import frc.robot.components.PIDMotorIOSparkMax;

public class ShooterSubsystem extends SubsystemBase{
    
    private final PIDMotor flywheelRight; //leader
    private final PIDMotor flywheelLeft;
    private final PIDMotor feeder; 
    private final PIDMotor hopper;
    private double desiredRPM;
    private LinearInterpolator interpolator;

    public ShooterSubsystem() {
        flywheelRight = new PIDMotor(new PIDMotorIOSparkMax(ShooterConstants.FLYWHEEL_RIGHT_ID, Shooter.FLYWHEEL_RIGHT_CONFIG));
        flywheelLeft = new PIDMotor(new PIDMotorIOSparkMax(ShooterConstants.FLYWHEEL_LEFT_ID, Shooter.FLYWHEEL_LEFT_CONFIG));
        feeder = new PIDMotor(new PIDMotorIOSparkMax(ShooterConstants.FEEDER_ID, Shooter.FEEDER_CONFIG));
        hopper = new PIDMotor(new PIDMotorIOSparkMax(ShooterConstants.HOPPER_ID, Shooter.HOPPER_CONFIG));
    }

    public void setDesiredRPM(double desiredRPM) {
        this.desiredRPM = desiredRPM;
        SmartDashboard.putNumber("Desired RPM", desiredRPM);
        Logger.recordOutput("Shooter/Desired RPM", desiredRPM);

    }

    public void setFeeder(double speed) {
        feeder.set(speed);
        Logger.recordOutput("Shooter/Feeder", speed);
    }

    public void setHopper(double speed) {
        hopper.set(speed);
        Logger.recordOutput("Shooter/Hopper", speed);
    }

    public void stopMotors() {
        flywheelRight.stopMotors();
        flywheelLeft.stopMotors();
        feeder.stopMotors();
        hopper.stopMotors();
    }

    @Override
    public void periodic() {
        if(desiredRPM == 0 && Math.abs(flywheelRight.getRPM()) < 150) {
            flywheelRight.set(0);
        } else {
            flywheelRight.setVelocity(desiredRPM, 0.00020352); 
        }
        SmartDashboard.putNumber("Actual RPM", flywheelRight.getRPM());
        Logger.recordOutput("RPM/Actual", flywheelRight.getRPM());
    }
}
