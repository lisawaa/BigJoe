package frc.robot.components;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class PIDMotorIOSparkMax implements PIDMotorIO {

  /** Motors */ 
  private SparkMax motor = null;

  /** Encoders */
  private RelativeEncoder encoder = null;

  /** Closed-loop controllers */
  private SparkClosedLoopController controller = null;

  /* Talks to a drive motor consisting of NEOs, SPARKS MAX, and a Through Bore Encoder.*/
  public PIDMotorIOSparkMax(int ID, SparkMaxConfig config) {
    // construct motor objects
    motor = new SparkMax(ID, MotorType.kBrushless);

    // construct encoder objects
    encoder = motor.getEncoder();

    // construct controller objects
    controller = motor.getClosedLoopController();
    
    // apply the passed in configuration
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /*  ------------------------------------------
      implementation of PIDMotorIO interface
      ------------------------------------------ */

    /** Read the current state (called during periodic update). */
    @Override public void updateInputs(PIDMotorIOInputsAutoLogged inputs) {
        inputs.RPM = encoder.getVelocity();
    }

    /** Reset drive encoder to zero. */
    @Override public void resetEncoder() {
        encoder.setPosition(0);
    }

    @Override public void setSetpoint(double setpoint, double FF) {
        controller.setSetpoint(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, FF);
    }

    @Override public void setVelocity(double RPM, double FF) {
        controller.setSetpoint(RPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0, FF);
    }

    @Override public void set(double speed){
        motor.set(speed);
    }

    @Override public void setVoltage(double voltage){
        motor.setVoltage(voltage);
    }

    @Override public void stopMotors(){
        motor.stopMotor();
    }
}