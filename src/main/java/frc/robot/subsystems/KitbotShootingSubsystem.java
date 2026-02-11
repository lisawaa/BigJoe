package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooting.ShootingConstants;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class KitbotShootingSubsystem extends SubsystemBase {
    private SparkMax shootingMotor;

    public KitbotShootingSubsystem() {
        shootingMotor = new SparkMax(ShootingConstants.SHOOTING_MOTOR_PORT, MotorType.kBrushed);
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(50);
        shootingMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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