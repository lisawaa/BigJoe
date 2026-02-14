package frc.robot.components;

import org.littletonrobotics.junction.AutoLog;

public interface PIDMotorIO {
     /** Inputs from the drive motor */
    @AutoLog
    public class PIDMotorIOInputs {
        public double RPM = 0.0;
        public double rotation = 0.0;
        //add the rest
    }

    /** Read the current state (called during periodic update). */
    default void updateInputs(PIDMotorIOInputsAutoLogged inputs) {}

    /** Reset drive encoder to zero. */
    default public void resetEncoder() {}

    default public void setSetpoint(double setpoint, double FF) {}

    default public void setVelocity(double RPM, double FF){}

    default public void set(double speed){}

    default public void setVoltage(double voltage){}

    default public void stopMotors(){}
}
