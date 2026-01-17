package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake.IntakeConstants;


public class KitbotIntakeSubsystem extends SubsystemBase {
    // define vars
    private Boolean isIntaking = false;
    private SparkMax intakeMotor;

    /**
     * Creates a new KitbotIntakeSubsystem.
     * Sets up SmartDashboard values for intake voltages
     * 
     * @return void
     */
    public KitbotIntakeSubsystem() {
        SmartDashboard.putNumber("Intake Feed Voltage", IntakeConstants.INTAKE_FEED_VOLT);

        intakeMotor = new SparkMax(IntakeConstants.INTAKE_MOTOR_PORT, MotorType.kBrushless);
        
        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.smartCurrentLimit(IntakeConstants.INTAKE_FEED_LIMIT);
        
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    /**
     * Sets the voltage of the intake roller motor
     *
     * @param voltage The voltage to set the intake roller motor to
     */
    public void setFeederRoller(double voltage) {
        intakeMotor.setVoltage(voltage);
    }

    /**
    * Returns whether the intake is currently intaking or not
    *
    * @Return {@Boolean} isIntaking
    */
    public Boolean isIntaking() {
        return isIntaking;
    }

    /**
     * Runs the intake motors to intake the fuel
     * 
     * @return void
     */
    public void Intake() {
        isIntaking = true;

        intakeMotor.setVoltage(IntakeConstants.INTAKE_FEED_VOLT);
        SmartDashboard.putBoolean("Is Intaking", isIntaking);
    }

    /**
     * Stops intake motors
     * 
     * @return void
     */
    public void stopIntake() {
        isIntaking = false;

        intakeMotor.setVoltage(0.0);
        SmartDashboard.putBoolean("Is Intaking", isIntaking);
    }

    /**
     * Stop intaking with a wpilib single run Command
     * 
     * @return {@edu.wpi.first.wpilibj2.command.Command} Command to stop intake
     */

    public Command stopIntakeCommand() {
        return runOnce(() -> {
            stopIntake();
        });
    }

    /**
     * Run intake with a wpilib single run Command
     * 
     * @return {@edu.wpi.first.wpilibj2.command.Command} Command to run intake
     */
    public Command runIntakeCommand() {
        return runOnce(() -> {
            Intake();
        });
    }

    @Override
    public void periodic() {

    }
}
