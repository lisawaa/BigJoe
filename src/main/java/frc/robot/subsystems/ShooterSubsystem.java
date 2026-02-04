package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Configs.Shooter;
import frc.robot.Constants.IDs.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase{
    
    private final SparkMax flywheel;
    private final RelativeEncoder flywheelEncoder;
    private final SparkClosedLoopController flywheelController;

    public ShooterSubsystem() {
        flywheel = new SparkMax(ShooterConstants.FLYWHEEL_ID, MotorType.kBrushless);
        flywheel.configure(Shooter.FLYWHEEL_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        flywheelEncoder = flywheel.getEncoder();
        flywheelController = flywheel.getClosedLoopController();
    }

    public void setRPM(double rpm) {
        flywheelController.setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0, .0003398478);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Target RPM", flywheelController.getSetpoint());
        SmartDashboard.putNumber("Actual RPM", flywheelEncoder.getVelocity());
    }

}
