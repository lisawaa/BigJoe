package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Configs.Intake;
import frc.robot.Constants.IDs.IntakeConstants;
import frc.robot.components.PIDMotor;
import frc.robot.components.PIDMotorIOSparkMax;

public class IntakeSubsystem extends SubsystemBase {
    
    private final PIDMotor intakeMotor;
    private final PIDMotor rotateMotor;

    public IntakeSubsystem(){
        intakeMotor = new PIDMotor(new PIDMotorIOSparkMax(IntakeConstants.INTAKE_ID, Intake.INTAKE_CONFIG));
        rotateMotor = new PIDMotor(new PIDMotorIOSparkMax(IntakeConstants.ROTATE_ID, Intake.ROTATE_CONFIG));
    }
}
