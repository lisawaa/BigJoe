package frc.robot.components;

import org.littletonrobotics.junction.AutoLog;

public interface PIDMotorIO {
    @AutoLog
    public class PIDMotorIOInputs {
        public double RPM = 0.0;
        public double rotation = 0.0;
        //add the rest
    }

    default void updateInputs(PIDMotorIOInputsAutoLogged inputs) {}

    default public void resetEncoder() {}

    default public void setSetpoint(double setpoint, double FF) {}

    default public void setVelocity(double RPM, double FF){}

    default public void set(double speed){}

    default public void setVoltage(double voltage){}

    default public void stopMotors(){}

    default public double getEncoder(){return 0;}
}
