package frc.robot.Constants;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public final class Configs {
    public static final class SwerveModule {
        public static final SparkMaxConfig TURNING_CONFIG = new SparkMaxConfig();
        public static final SparkMaxConfig FL_CONFIG = new SparkMaxConfig();
        public static final SparkMaxConfig FR_CONFIG = new SparkMaxConfig();
        public static final SparkMaxConfig BL_CONFIG = new SparkMaxConfig();
        public static final SparkMaxConfig BR_CONFIG = new SparkMaxConfig();
      
        public static final class Sim {
            // NEO motor simulation constants based on actual hardware specs
            // NEO free speed: 5676 RPM, Stall torque: 3.36 N⋅m @ 12V, Stall current: 166A

            // Drive motor constants (linear motion)
            // kV = nominal_voltage / max_linear_speed = 12V / 4.46 m/s
            public static final double DRIVE_VOLTS_PER_VELOCITY = 12.0 / Drive.Constants.MAX_METERS_PER_SECOND; // ~2.69 V/(m/s)

            // kA calculated from NEO stall torque, gear ratio, wheel radius, and estimated robot mass
            // With 4 drive motors, stall force ~1791N, mass ~50kg gives ~35.8 m/s² theoretical max
            // Practical kA accounts for friction, mass distribution: 12V / practical_accel
            public static final double DRIVE_VOLTS_PER_ACCELERATION = 0.27; // V/(m/s²)

            // Turn motor constants (rotational motion at module output shaft)
            // Max output angular velocity: 5676 RPM / 46.42 = 122.3 RPM = 12.8 rad/s
            // kV = 12V / 12.8 rad/s
            public static final double TURN_VOLTS_PER_VELOCITY = 0.938; // V/(rad/s)

            // kA based on output torque and estimated module inertia (~0.02 kg⋅m²)
            // Max angular accel: (3.36 N⋅m * 46.42) / 0.02 = ~7800 rad/s²
            public static final double TURN_VOLTS_PER_ACCELERATION = 0.0015; // V/(rad/s²)

            // Actual MAXSwerve gear ratios
            public static final double DRIVE_GEAR_RATIO = Drive.ModuleConstants.DRIVE_MOTOR_REDUCTION; // ~5.077:1
            public static final double TURN_GEAR_RATIO = 46.42; // MAXSwerve steering reduction (150/7 * 10/1 worm)

            public static final double MAX_LINEAR_SPEED = Drive.Constants.MAX_METERS_PER_SECOND; // 4.46 m/s
            public static final int DRIVE_MOTOR_COUNT = 1; // 1 NEO motor per module
            public static final int TURN_MOTOR_COUNT = 1; // 1 NEO motor per steering

            // Simulation PID values match actual hardware configuration
            public static final double TURN_SIM_PID_P = 1.0;  // Matches TURNING_CONFIG.pid
            public static final double TURN_SIM_PID_I = 0.0;
            public static final double TURN_SIM_PID_D = 0.0;
            public static final double DRIVE_SIM_PID_P = 0.04; // Matches drive configs.pid
            public static final double DRIVE_SIM_PID_I = 0.0;
            public static final double DRIVE_SIM_PID_D = 0.0;

            public static final double SIM_TICK_TIME = 0.02; // seconds (50Hz)
        }

        static {
            //Module constants used to calculate conversion factors and feed forward gain.
            double DRIVING_FACTOR = Drive.ModuleConstants.WHEEL_DIAMETER * Math.PI
                / Drive.ModuleConstants.DRIVE_MOTOR_REDUCTION;
            double FF_VELOCITY = 1 / Drive.ModuleConstants.DRIVE_WHEEL_FREE_RPS;
            double TURNING_FACTOR = 2 * Math.PI;

            //Update as needed
            TURNING_CONFIG
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(20);
            TURNING_CONFIG.absoluteEncoder
                .inverted(true)
                .positionConversionFactor(TURNING_FACTOR)
                .velocityConversionFactor(TURNING_FACTOR / 60.0);
            TURNING_CONFIG.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(1,0,0) //update
                .outputRange(-1, 1)
                //PID wrap around optimizes angle for turning
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, TURNING_FACTOR);

            FL_CONFIG
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50)
                .inverted(false);
            FL_CONFIG.encoder
                .positionConversionFactor(DRIVING_FACTOR) //meters
                .velocityConversionFactor(DRIVING_FACTOR / 60.0);
            FL_CONFIG.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.04, 0,0)
                .apply(new FeedForwardConfig().kV(FF_VELOCITY))
                .outputRange(-1, 1);

            FR_CONFIG
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50)
                .inverted(!false);
            FR_CONFIG.encoder
                .positionConversionFactor(DRIVING_FACTOR) //meters
                .velocityConversionFactor(DRIVING_FACTOR / 60.0);
            FR_CONFIG.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.04, 0,0)
                .apply(new FeedForwardConfig().kV(FF_VELOCITY))
                .outputRange(-1, 1);

            BL_CONFIG
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50)
                .inverted(false);
            BL_CONFIG.encoder
                .positionConversionFactor(DRIVING_FACTOR) //meters
                .velocityConversionFactor(DRIVING_FACTOR / 60.0);
            BL_CONFIG.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.04, 0,0)
                .apply(new FeedForwardConfig().kV(FF_VELOCITY))
                .outputRange(-1, 1);

            BR_CONFIG
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50)
                .inverted(!true);
            BR_CONFIG.encoder
                .positionConversionFactor(DRIVING_FACTOR) //meters
                .velocityConversionFactor(DRIVING_FACTOR / 60.0);
            BR_CONFIG.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.04, 0,0)
                .apply(new FeedForwardConfig().kV(FF_VELOCITY))
                .outputRange(-1, 1);
        }
    }

    public static final class Climb {
        public static final SparkMaxConfig INNER_LEFT_CONFIG = new SparkMaxConfig();
        public static final SparkMaxConfig INNER_RIGHT_CONFIG = new SparkMaxConfig();
        public static final SparkMaxConfig OUTER_LEFT_CONFIG = new SparkMaxConfig();
        public static final SparkMaxConfig OUTER_RIGHT_CONFIG = new SparkMaxConfig();

        //invert if needed
        static {
            INNER_LEFT_CONFIG
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50);
            INNER_RIGHT_CONFIG
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50)
                .follow(IDs.ClimbConstants.INNER_LEFT_ID);
            OUTER_LEFT_CONFIG
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50);
            OUTER_RIGHT_CONFIG
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50)
                .follow(IDs.ClimbConstants.OUTER_LEFT_ID);       
        }
    }

    public static final class Shooter {
        public static final SparkMaxConfig FLYWHEEL_CONFIG = new SparkMaxConfig();
        public static final SparkMaxConfig SECONDARY_CONFIG = new SparkMaxConfig();


        //invert if needed
        static {
            double FLYWHEEL_FACTOR = 1; //change/delete?

            FLYWHEEL_CONFIG
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(50)
                .inverted(false); 
           
          //  FLYWHEEL_CONFIG.encoder
          //      .positionConversionFactor(FLYWHEEL_FACTOR) //meters
          //      .velocityConversionFactor(FLYWHEEL_FACTOR / 60.0);

            FLYWHEEL_CONFIG.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.0004, 0, 0)
                .outputRange(-1, 1)
            .maxMotion
                // Set MAXMotion parameters for position control - Edit
                .allowedProfileError(100);

            SECONDARY_CONFIG
                .idleMode(IdleMode.kCoast)
                .inverted(true)
                .smartCurrentLimit(50)
                .inverted(false);
        }
    }

    public static final class Intake {
        public static final SparkMaxConfig INTAKE_CONFIG = new SparkMaxConfig();
        public static final SparkMaxConfig ROTATE_CONFIG = new SparkMaxConfig();


        //invert if needed
        static {
            INTAKE_CONFIG
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(50);
            INTAKE_CONFIG
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(50);
            ROTATE_CONFIG
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50);
            ROTATE_CONFIG
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50);
        }
    }

}
