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
        public static final double FF_VELOCITY = 1 / Drive.ModuleConstants.DRIVE_WHEEL_FREE_RPS; 
        
        public static final class Sim {
            public static final double VOLTS_PER_VELOCITY = 1.0 / 4.5; // Volts per (meter per second) - kV
            public static final double VOLTS_PER_ACCELERATION = 0.23; // Volts per (meter per second squared) - kA
            public static final int DRIVE_GEAR_RATIO = 3;
            public static final int TURN_GEAR_RATIO = 3;
            public static final double MAX_LINEAR_SPEED = 4.5; // m/s
            public static final int DRIVE_MOTOR_COUNT = 1; // 1 NEO motor
            public static final int TURN_MOTOR_COUNT = 1; // 1 NEO motor

            public static final double TURN_SIM_PID_P = 8.0;
            public static final double TURN_SIM_PID_I = 0.0;
            public static final double TURN_SIM_PID_D = 0.0;
            public static final double DRIVE_SIM_PID_P = 2.0;  
            public static final double DRIVE_SIM_PID_I = 0.0;
            public static final double DRIVE_SIM_PID_D = 0.0;

            public static final double SIM_TICK_TIME = 0.02; // seconds (50Hz)
        }

        static {
            //Module constants used to calculate conversion factors and feed forward gain.
            double DRIVING_FACTOR = Drive.ModuleConstants.WHEEL_DIAMETER * Math.PI
                / Drive.ModuleConstants.DRIVE_MOTOR_REDUCTION;
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
}
