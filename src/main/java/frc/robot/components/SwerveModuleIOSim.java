package frc.robot.components;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.Configs;

public class SwerveModuleIOSim implements SwerveModuleIO {
    // Simulated motor physics
    private final DCMotorSim drivePhysSim;
    private final DCMotorSim turnPhysSim;
    
    // PID controllers for simulating closed-loop control
    private final PIDController driveSimPID;
    private final PIDController turnSimPID;
    
    private SwerveModuleState targetState = new SwerveModuleState();
    private final double chassisAngularOffset;
    
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;
    private double driveEncoderOffset = 0.0;
    
    public SwerveModuleIOSim(double angularOffset) {
        drivePhysSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                Configs.SwerveModule.Sim.VOLTS_PER_VELOCITY, 
                Configs.SwerveModule.Sim.VOLTS_PER_ACCELERATION),
            DCMotor.getNEO(Configs.SwerveModule.Sim.DRIVE_MOTOR_COUNT)
        );
        
        turnPhysSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                Configs.SwerveModule.Sim.VOLTS_PER_VELOCITY,
                Configs.SwerveModule.Sim.VOLTS_PER_ACCELERATION),
            DCMotor.getNEO(Configs.SwerveModule.Sim.TURN_MOTOR_COUNT)
        );
        
        driveSimPID = new PIDController(
            Configs.SwerveModule.Sim.DRIVE_SIM_PID_P,
            Configs.SwerveModule.Sim.DRIVE_SIM_PID_I,
            Configs.SwerveModule.Sim.DRIVE_SIM_PID_D);
        turnSimPID = new PIDController(
            Configs.SwerveModule.Sim.TURN_SIM_PID_P,
            Configs.SwerveModule.Sim.TURN_SIM_PID_I,
            Configs.SwerveModule.Sim.TURN_SIM_PID_D);
        turnSimPID.enableContinuousInput(-Math.PI, Math.PI);
        
        chassisAngularOffset = angularOffset;
    }
    
    @Override
    public void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
        // Update simulated physics
        drivePhysSim.update(Configs.SwerveModule.Sim.SIM_TICK_TIME);
        turnPhysSim.update(Configs.SwerveModule.Sim.SIM_TICK_TIME);
        
        inputs.drivePositionMeters = (drivePhysSim.getAngularPositionRad() - driveEncoderOffset) / (2 * Math.PI);
        inputs.driveVelocityMetersPerSec = drivePhysSim.getAngularVelocityRadPerSec();
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = drivePhysSim.getCurrentDrawAmps();
        
        inputs.turnPositionRad = turnPhysSim.getAngularPositionRad() - chassisAngularOffset;
        inputs.turnVelocityRadPerSec = turnPhysSim.getAngularVelocityRadPerSec();
        inputs.turnAppliedVolts = turnAppliedVolts;
        inputs.turnCurrentAmps = turnPhysSim.getCurrentDrawAmps();
    }
    
    @Override
    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState correctedState = new SwerveModuleState();
        correctedState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));
        
        correctedState.optimize(new Rotation2d(turnPhysSim.getAngularPositionRad()));
        targetState = correctedState;
        
        // Simulate closed-loop control
        driveAppliedVolts = driveSimPID.calculate(
            drivePhysSim.getAngularVelocityRadPerSec(), 
            targetState.speedMetersPerSecond
        );
        turnAppliedVolts = turnSimPID.calculate(
            turnPhysSim.getAngularPositionRad(), 
            targetState.angle.getRadians()
        );
        
        driveAppliedVolts = MathUtil.clamp(driveAppliedVolts, -12.0, 12.0);
        turnAppliedVolts = MathUtil.clamp(turnAppliedVolts, -12.0, 12.0);
        
        drivePhysSim.setInputVoltage(driveAppliedVolts);
        turnPhysSim.setInputVoltage(turnAppliedVolts);
    }
    
    @Override
    public void resetDriveEncoder() {
        driveEncoderOffset = drivePhysSim.getAngularPositionRad();
    }  
}
