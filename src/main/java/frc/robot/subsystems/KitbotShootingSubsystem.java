package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooting.ShootingConstants;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class KitbotShootingSubsystem extends SubsystemBase{
    private TalonFX shootingMotor;

    public KitbotShootingSubsystem() {
        shootingMotor = new TalonFX(ShootingConstants.SHOOTING_MOTOR_PORT);
        SmartDashboard.putNumber("Shooting voltage value", ShootingConstants.SHOOTING_VOLTAGE);
    }

    public void shooting() {
        shootingMotor.setVoltage(ShootingConstants.SHOOTING_VOLTAGE);
    }

    public void stopMotors() {
        shootingMotor.setVoltage(0.0);
    }

    @Override
    public void periodic() {}
}