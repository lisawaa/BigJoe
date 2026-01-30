package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake.IntakeConstants;


public class KitbotIntakeSubsystem extends SubsystemBase {
    private TalonFX intakeMotor;

    public KitbotIntakeSubsystem() {
        intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_PORT);
        SmartDashboard.putNumber("Intake Feed Voltage", IntakeConstants.INTAKE_FEED_VOLT);
    }

    public void setFeederRoller(double voltage) {
        intakeMotor.setVoltage(voltage);
    }

    public void intake() {
        intakeMotor.setVoltage(IntakeConstants.INTAKE_FEED_VOLT);
    }

    public void reverse() {
        intakeMotor.setVoltage(IntakeConstants.INTAKE_REVERSE_VOLT);
    }

    public void stopIntake() {
        intakeMotor.setVoltage(0.0);
    }

    @Override
    public void periodic() {}
}
