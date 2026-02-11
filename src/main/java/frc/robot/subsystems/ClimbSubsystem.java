package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Climb;
import frc.robot.Constants.Configs;
import frc.robot.Constants.IDs;

public class ClimbSubsystem extends SubsystemBase{
    
    private final SparkMax innerLeft;
    private final SparkMax innerRight;
    private final SparkMax outerLeft;
    private final SparkMax outerRight;

    private final RelativeEncoder innerEncoder;
    private final RelativeEncoder outerEncoder;

    //add limit switches?
    public ClimbSubsystem() {
        innerLeft = new SparkMax(IDs.ClimbConstants.INNER_LEFT_ID, MotorType.kBrushless);
        innerRight = new SparkMax(IDs.ClimbConstants.INNER_RIGHT_ID, MotorType.kBrushless);
        outerLeft = new SparkMax(IDs.ClimbConstants.OUTER_LEFT_ID, MotorType.kBrushless);
        outerRight = new SparkMax(IDs.ClimbConstants.OUTER_RIGHT_ID, MotorType.kBrushless);

        innerLeft.configure(Configs.Climb.INNER_LEFT_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        innerRight.configure(Configs.Climb.INNER_RIGHT_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        outerLeft.configure(Configs.Climb.OUTER_LEFT_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        outerRight.configure(Configs.Climb.OUTER_RIGHT_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        innerEncoder = innerLeft.getEncoder();
        outerEncoder = outerLeft.getEncoder();
        resetEncoders();
    }

    public void moveInner(double speed) {
        //if bot-limit switch
        //speed = Math.max(speed, 0) <- prevents motors from impossibly going down

        innerLeft.set(speed);
    }

    public void moveOuter(double speed) {
        //if bot-limit switch
        //speed = Math.max(speed, 0) <- prevents motors from impossibly going down

        outerLeft.set(speed);
    }

    public void resetEncoders() {
        innerEncoder.setPosition(0);
        outerEncoder.setPosition(0);
    }

    public void stop() {
        innerLeft.stopMotor();
        outerLeft.stopMotor();
    }

    @Override
    public void periodic () {
        SmartDashboard.putNumber("Inner Encoder: ", Climb.Constants.INNER_ENCODER_REVERSED * innerEncoder.getPosition());
        SmartDashboard.putNumber("Outer Encoder: ", Climb.Constants.OUTER_ENCODER_REVERSED * outerEncoder.getPosition());
        //Add limit switches and whether robot is in climb range

        Logger.recordOutput("Climb/Position/Inner", Climb.Constants.INNER_ENCODER_REVERSED * innerEncoder.getPosition());
        Logger.recordOutput("Climb/Position/Outer", Climb.Constants.OUTER_ENCODER_REVERSED * outerEncoder.getPosition());
        //Add limit switches and whether robot is in climb range
    }
}
