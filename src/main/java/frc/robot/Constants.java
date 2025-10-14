package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.SwerveModule.ModuleConfiguration;

public class Constants {
    public static class Drivetrain {

        public static final double FB_LENGTH = 0.6985;
        public static final double LR_LENGTH = 0.6223;

        public static ModuleConfiguration FL_CONFIG = new ModuleConfiguration();
        public static ModuleConfiguration FR_CONFIG = new ModuleConfiguration();
        public static ModuleConfiguration BL_CONFIG = new ModuleConfiguration();
        public static ModuleConfiguration BR_CONFIG = new ModuleConfiguration();

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
