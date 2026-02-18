package frc.robot.utils;

import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.wpilibj.DriverStation;

public class ApplyConfig {
    public static StatusCode applyConfigWithRetry(String motorRole, String moduleName, java.util.concurrent.Callable<StatusCode> applier) {
        final int maxAttempts = 5;
        final long backoffMs = 25;
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int attempt = 1; attempt <= maxAttempts; attempt++) {
            try {
                status = applier.call();
            } catch (Exception e) {
                DriverStation.reportError("Exception while applying " + motorRole + " config for module " + moduleName + ": " + e.getMessage(), false);
                status = StatusCode.GeneralError;
            }
            if (status.isOK()) {
                return status;
            }
            try {
                Thread.sleep(backoffMs);
            } catch (InterruptedException ie) {
                Thread.currentThread().interrupt();
                break;
            }
        }
        return status;
    }
}
