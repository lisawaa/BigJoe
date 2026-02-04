package frc.robot.components;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
    /** Inputs from the drive motor */
    @AutoLog
    public class SwerveModuleIOInputs {
        public double drivePositionMeters = 0.0;
        public double driveVelocityMetersPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public double turnPositionRad = 0.0;
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;
    }

    /** Read the current state (called during periodic update). */
    default void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {}

    /** Set desired state using hardware closed-loop control. */
    default public void setDesiredState(SwerveModuleState state) {}

    /** Reset drive encoder to zero. */
    default public void resetDriveEncoder() {}
}
