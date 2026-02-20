package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.SwerveModule.ModuleConfiguration;

public class Constants {
    public static final double TICK_PER_SECOND = 50.0;
    public static final double EPSILON = 1e-9;

    public static class Drivetrain {

        public static class Hardware {
            public static final double WHEEL_DIAMETER_METER = 0.1016; // 4 inches

            //public static final double DRIVE_SENSOR_TO_MECHANISM_RATIO = 40500.0 / 5760.0; // around 7.03
            public static final double DRIVE_SENSOR_TO_MECHANISM_RATIO = 6.03;
            public static final double DRIVE_ROTOR_TO_SENSOR_RATIO = 1.0;

            public static final double STEER_SENSOR_TO_MECHANISM_RATIO = 1.0 / 1.0;
            public static final double STEER_ROTOR_TO_SENSOR_RATIO = 287.0 / 11.0; // around 26.09
        }

        public static final double FB_LENGTH = 0.55245; // 21.75 inches
        public static final double LR_LENGTH = 0.55245; // 21.75 inches

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 5.0;

        public static final double MAX_TRANSLATOIN_ACCELERATION_METERS_PER_SECOND_SQUARED = 15.0;
        public static final double MAX_LATERAL_JERK_RADIANS_PER_SECOND_SQUARED = 20;
        public static final double MAX_ROTATION_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 300;

        public static final boolean PIGEON_INVERTED = false;
        public static final boolean STEER_INVERTED = false;

        public static final ModuleConfiguration FL_CONFIG = new ModuleConfiguration();
        public static final ModuleConfiguration FR_CONFIG = new ModuleConfiguration();
        public static final ModuleConfiguration BL_CONFIG = new ModuleConfiguration();
        public static final ModuleConfiguration BR_CONFIG = new ModuleConfiguration();

        static {FL_CONFIG.moduleName = "Front Left";
            FL_CONFIG.index = 0;
            FL_CONFIG.position = new Translation2d(FB_LENGTH / 2, LR_LENGTH / 2); // +,+
            FL_CONFIG.encoderOffset = Rotation2d.fromRotations(0.0955);}
        static {FR_CONFIG.moduleName = "Front Right";
            FR_CONFIG.index = 1;
            FR_CONFIG.position = new Translation2d(FB_LENGTH / 2, -LR_LENGTH / 2); // +,-
            FR_CONFIG.encoderOffset = Rotation2d.fromRotations(0.2302);}
        static {BL_CONFIG.moduleName = "Back Left";
            BL_CONFIG.index = 2;
            BL_CONFIG.position = new Translation2d(-FB_LENGTH / 2, LR_LENGTH / 2); // -,+
            BL_CONFIG.encoderOffset = Rotation2d.fromRotations(-0.1323);}
        static {BR_CONFIG.moduleName = "Back Right";
            BR_CONFIG.index = 3;
            BR_CONFIG.position = new Translation2d(-FB_LENGTH / 2, -LR_LENGTH / 2); // -,-
            BR_CONFIG.encoderOffset = Rotation2d.fromRotations(-0.0169);}

        public static final double STEER_KP = 1.8;
        public static final double STEER_KI = 0.0;
        public static final double STEER_KD = 0.02;
        public static final double STEER_PEAK_VOLTAGE = 14.0;
        
        public static final double DRIVE_KS = 0.1;
        public static final double DRIVE_KV = 0.12;
        public static final double DRIVE_KP = 0.2;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.07;
        public static final double DRIVE_PEAK_VOLTAGE = 14.0;

        public static final double HEADING_COEFF = 3.0;
        public static final double HEADING_KP = 0.8;
        public static final double HEADING_KI = 0.0;
        public static final double HEADING_KD = 0.4;

        public static class Auto {
            public static final double TRANSLATION_KP = 5.0;
            public static final double TRANSLATION_KI = 0.0;
            public static final double TRANSLATION_KD = 0.0;

            public static final double ROTATION_KP = 5.0;
            public static final double ROTATION_KI = 0.0;
            public static final double ROTATION_KD = 0.0;
        }
    }

    public static class Shooter {
        public static final double SHOOTER_KP = 0.1;
        public static final double SHOOTER_KI = 0.0;
        public static final double SHOOTER_KD = 0.0;
        public static final double SHOOTER_KV = 0.1;

        public static final double HOOD_KP = 0.3;
        public static final double HOOD_KI = 0.0;
        public static final double HOOD_KD = 0.0;

        public static final double NEAR_DISTANCE = 2.0;
        public static final double FAR_DISTANCE = 5.0;

        public static class MotorConfig {
            public static final int SHOOTER_STATOR_CURRENT_LIMIT = 60;
            public static final int SHOOTER_SUPPLY_CURRENT_LIMIT = 70;
        }

        public static class Hardware {
            public static final double HOOD_SENSOR_TO_MECHANISM_RATIO = 1.0 / 1.0;
            public static final double HOOD_ROTOR_TO_SENSOR_RATIO = 1.0 / 1.0; // around 25.45

            public static final double HOOD_MAX = 0.4;
            public static final double HOOD_MIN = -2.3;

            public static final double SHOOTER_ACCURACY_TOLERANCE = 0.2; // RPS
            public static final double HOOD_ACCURACY_TOLERANCE = 0.005; // Rotations
        }
    }

    public static class Indexer {
        public static class MotorConfig {
            public static final int INDEXER_STATOR_CURRENT_LIMIT = 40;
            public static final int INDEXER_SUPPLY_CURRENT_LIMIT = 50;
        }
    }

    public static class Intake {
        public static class MotorConfig {
            public static final int INTAKE_ARM_STATOR_CURRENT_LIMIT = 30;
            public static final int INTAKE_ARM_SUPPLY_CURRENT_LIMIT = 40;
            public static final int INTAKE_WHEEL_STATOR_CURRENT_LIMIT = 30;
            public static final int INTAKE_WHEEL_SUPPLY_CURRENT_LIMIT = 40;
        }

        public static final double INTAKE_WHEEL_KP = 0.1;
        public static final double INTAKE_WHEEL_KI = 0.0;
        public static final double INTAKE_WHEEL_KD = 0.02;

        public static final double INTAKE_ARM_KP = 0.2;
        public static final double INTAKE_ARM_KI = 0.0;
        public static final double INTAKE_ARM_KD = 0.02;

        public static class Hardware {
            public static final double INTAKE_ARM_SENSOR_TO_MECHANISM_RATIO = 1.0 / 1.0;
            public static final double INTAKE_ARM_ROTOR_TO_SENSOR_RATIO = 1.0 / 1.0;

            public static final double INTAKE_ARM_MAX = 0.5;
            public static final double INTAKE_ARM_MIN = -0.5;

            public static final double INTAKE_ARM_ACCURACY_TOLERANCE = 0.2; // Rotations

            public static final double INTAKE_ARM_INTAKE_POSITION = INTAKE_ARM_MIN;
            public static final double INTAKE_ARM_RETRACT_POSITION = INTAKE_ARM_MAX;

            public static final double INTAKE_WHEEL_POWER = 1.0;
            public static final double REVERSE_INTAKE_WHEEL_POWER = -INTAKE_WHEEL_POWER;
            public static final double RETRACT_WHEEL_POWER = 0.0;
        }
    }
}
