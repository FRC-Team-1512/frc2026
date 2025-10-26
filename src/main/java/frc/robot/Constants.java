package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.SwerveModule.ModuleConfiguration;

public class Constants {
    public static final double TICK_PER_SECOND = 50.0;

    public static class Drivetrain {

        public static class Hardware {
            public static final double WHEEL_DIAMETER_METER = 0.1016;

            public static final double DRIVE_SENSOR_TO_MECHANISM_RATIO = 40500.0 / 5760.0;
            public static final double DRIVE_ROTOR_TO_SENSOR_RATIO = 1.0;

            public static final double STEER_SENSOR_TO_MECHANISM_RATIO = 287.0 / 11.0;
            public static final double STEER_ROTOR_TO_SENSOR_RATIO = 1.0;
        }

        public static final double FB_LENGTH = 0.6985;
        public static final double LR_LENGTH = 0.6223;

        public static final int PIGEON_INVERTED = 1; // 1 for not inverted, -1 for inverted

        public static final ModuleConfiguration FL_CONFIG = new ModuleConfiguration();
        public static final ModuleConfiguration FR_CONFIG = new ModuleConfiguration();
        public static final ModuleConfiguration BL_CONFIG = new ModuleConfiguration();
        public static final ModuleConfiguration BR_CONFIG = new ModuleConfiguration();

        static {FL_CONFIG.moduleName = "Front Left";
            FL_CONFIG.index = 0;
            FL_CONFIG.position = new Translation2d(FB_LENGTH / 2, LR_LENGTH / 2); // +,+
            FL_CONFIG.encoderInverted = false;
            FL_CONFIG.encoderOffset = Math.PI * (1.0 / 2.0);}
        static {FR_CONFIG.moduleName = "Front Left";
            FR_CONFIG.index = 1;
            FR_CONFIG.position = new Translation2d(FB_LENGTH / 2, -LR_LENGTH / 2); // +,-
            FR_CONFIG.encoderInverted = false;
            FR_CONFIG.encoderOffset = Math.PI * (1.0 / 2.0);}
        static {BL_CONFIG.moduleName = "Front Left";
            BL_CONFIG.index = 2;
            BL_CONFIG.position = new Translation2d(-FB_LENGTH / 2, LR_LENGTH / 2); // -,+
            BL_CONFIG.encoderInverted = false;
            BL_CONFIG.encoderOffset = Math.PI * (1.0 / 2.0);}
        static {BR_CONFIG.moduleName = "Front Left";
            BR_CONFIG.index = 3;
            BR_CONFIG.position = new Translation2d(-FB_LENGTH / 2, -LR_LENGTH / 2); // -,-
            BR_CONFIG.encoderInverted = false;
            BR_CONFIG.encoderOffset = Math.PI * (1.0 / 2.0);}
    }
}
