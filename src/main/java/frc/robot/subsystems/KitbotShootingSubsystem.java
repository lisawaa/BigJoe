package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.Shooting.ShootingConstants;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class KitbotShootingSubsystem extends SubsystemBase {
    private SparkMax shootingMotor;

    public KitbotShootingSubsystem() {
        shootingMotor = new SparkMax(ShootingConstants.MotorID, MotorType.kBrushless);
        SparkMaxConfig feederConfig = new SparkMaxConfig();
        feederConfig.smartCurrentLimit(ShootingConstants.SHOOTING_MOTOR_CURRENT_LIMIT);
        shootingMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        SmartDashboard.putNumber("Shooting voltage value", ShootingConstants.SHOOTING_VOLTAGE);
    }

    public void shooting() {
        shootingMotor.setVoltage(ShootingConstants.voltage);
    }

    public void stopMotors() {
        shootingMotor.setVoltage(0.0);
    }


}