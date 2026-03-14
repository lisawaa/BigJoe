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
    
    private final PIDMotor flywheel;
    private final PIDMotor secondary; //temp
    private LinearInterpolator rpmInterpolator;

    public ShooterSubsystem() {
        flywheel = new PIDMotor(new PIDMotorIOSparkMax(ShooterConstants.FLYWHEEL_ID, Shooter.FLYWHEEL_CONFIG));
        secondary = new PIDMotor(new PIDMotorIOSparkMax(ShooterConstants.SECONDARY_ID, Shooter.SECONDARY_CONFIG));
        rpmInterpolator = new LinearInterpolator(Shooter.points);
    }

    public void setRPM(double rpm) {
        if(rpm == 0) // && Math.abs(flywheel.getRPM()) < 200)
            flywheel.set(0);
        else
            flywheel.setVelocity(rpm, 0.00020352); //+ (rpm*0.04)
            // flywheel.setVelocity(rpm, rpmInterpolator.getOutput());
        SmartDashboard.putNumber("Desired RPM", rpm);
        Logger.recordOutput("RPM/Desired", rpm);
    }

    public void setSecondary(double speed) {
        secondary.set(speed);
    }

    public void stopMotors() {
        flywheel.stopMotors();
        secondary.stopMotors();
    }

    @Override
    public void periodic() {
        //SmartDashboard.putNumber("Target RPM", flywheelController.getSetpoint());
        SmartDashboard.putNumber("Actual RPM", flywheel.getRPM());
        Logger.recordOutput("RPM/Actual", flywheel.getRPM());

        //Logging Target vs Actual RPM
        //Logger.recordOutput("Shooter/TargetRPM", flywheelController.getSetpoint());
        //Logger.recordOutput("Shooter/ActualRPM", flywheelEncoder.getVelocity());

        //Logging Shooter Motor Current
        //Logger.recordOutput("Shooter/Flywheel/Current", flywheel.getOutputCurrent());

        //Logging Shooter Motor Temperature
        //Logger.recordOutput("Shooter/Flywheel/Temperature", flywheel.getMotorTemperature());

        //Logging Shooter Fuel Sensor (Might not be applicable)
    }
}
