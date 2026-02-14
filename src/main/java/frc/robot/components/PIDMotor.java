package frc.robot.components;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class PIDMotor extends SubsystemBase {

  private PIDMotorIO io;
  private final PIDMotorIOInputsAutoLogged inputs = new PIDMotorIOInputsAutoLogged();
   
  /* Constructs a SwerveModule */
  public PIDMotor(PIDMotorIO io) {
    this.io = io;
    io.resetEncoder();
  }

  @Override public void periodic() {
    // update the IO inputs
    io.updateInputs(inputs);
    //Add logging
  }

  public void setSetpoint(double setpoint, double FF) {
    io.setSetpoint(setpoint, FF);
    //Add logging
  }

  public void setVelocity(double RPM, double FF) {
    io.setSetpoint(RPM, FF);
    //Add logging
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
    //Add logging
  }

  public void set(double speed) {
    io.set(speed);
    //Add logging

  }

  public double getRPM() {
    return inputs.RPM;
  }

  //Stops the motor;
  public void stopMotors() {
    io.stopMotors();
    //Add logging
  }
}