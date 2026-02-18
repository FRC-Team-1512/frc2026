package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {

    private final TalonFX _steerMotor;
    private final TalonFX _driveMotor;
    private final CANcoder _encoder;

    private final PIDController _steerPositionSpeed;
    private final VelocityVoltage _driveVelocityVoltage;

    private final Translation2d _moduleLocation;
    private final int _moduleIndex;
    private SwerveModuleState _desiredModuleState;

    // ======================================================================================

    public SwerveModule(int steerPort, int drivePort, int encoderPort, ModuleConfiguration config) {

        _steerMotor = new TalonFX(steerPort);
        _driveMotor = new TalonFX(drivePort);
        _encoder = new CANcoder(encoderPort);

        // -------------------------------------------------------------------------------------

        TalonFXConfiguration _steerMotorConfig = new TalonFXConfiguration();

        if(Constants.Drivetrain.STEER_INVERTED) {
            _steerMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        } else {
            _steerMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        }

        _steerMotorConfig.Voltage.withPeakForwardVoltage(Volts.of(Constants.Drivetrain.STEER_PEAK_VOLTAGE))
            .withPeakReverseVoltage(Volts.of(-Constants.Drivetrain.STEER_PEAK_VOLTAGE));

        TalonFXConfiguration _driveMotorConfig = new TalonFXConfiguration();
        _driveMotorConfig.Feedback.SensorToMechanismRatio = Constants.Drivetrain.Hardware.DRIVE_SENSOR_TO_MECHANISM_RATIO;
        _driveMotorConfig.Feedback.RotorToSensorRatio = Constants.Drivetrain.Hardware.DRIVE_ROTOR_TO_SENSOR_RATIO;
        _driveMotorConfig.Slot0.kS = Constants.Drivetrain.DRIVE_KS;
        _driveMotorConfig.Slot0.kV = Constants.Drivetrain.DRIVE_KV;
        _driveMotorConfig.Slot0.kP = Constants.Drivetrain.DRIVE_KP;
        _driveMotorConfig.Slot0.kI = Constants.Drivetrain.DRIVE_KI;
        _driveMotorConfig.Slot0.kD = Constants.Drivetrain.DRIVE_KD;
        _driveMotorConfig.Voltage.withPeakForwardVoltage(Volts.of(Constants.Drivetrain.DRIVE_PEAK_VOLTAGE))
            .withPeakReverseVoltage(Volts.of(-Constants.Drivetrain.DRIVE_PEAK_VOLTAGE));
        
        CANcoderConfiguration _encoderConfig = new CANcoderConfiguration();

        if (Constants.Drivetrain.STEER_INVERTED) {
            _encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        }else {
            _encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        }

        _encoderConfig.MagnetSensor.MagnetOffset = config.encoderOffset.getRotations();

        // -------------------------------------------------------------------------------------

        _steerPositionSpeed = new PIDController(Constants.Drivetrain.STEER_KP, Constants.Drivetrain.STEER_KI, Constants.Drivetrain.STEER_KD);
        _steerPositionSpeed.enableContinuousInput(-0.5, 0.5);

        _driveVelocityVoltage = new VelocityVoltage(0).withSlot(0);

        // -------------------------------------------------------------------------------------

        StatusCode steerMotorStatus = applyConfigWithRetry("Steer", config.moduleName, () -> _steerMotor.getConfigurator().apply(_steerMotorConfig));
        if (!steerMotorStatus.isOK()) {
            DriverStation.reportError(
                "Failed to apply steer motor config for module " + config.moduleName + 
                " after retries. Status: " + steerMotorStatus,
                false
            );
        }
        StatusCode driveMotorStatus = applyConfigWithRetry("Drive", config.moduleName, () -> _driveMotor.getConfigurator().apply(_driveMotorConfig));
        if (!driveMotorStatus.isOK()) {
            DriverStation.reportError(
                "Failed to apply drive motor config for module " + config.moduleName + 
                " after retries. Status: " + driveMotorStatus,
                false
            );
        }
        StatusCode encoderStatus = applyConfigWithRetry("Encoder", config.moduleName, () -> _encoder.getConfigurator().apply(_encoderConfig));
        if (!encoderStatus.isOK()) {
            DriverStation.reportError("Failed to apply encoder config for module " + config.moduleName + 
                " after retries. Status: " + encoderStatus,
                false
            );
        }

        // -------------------------------------------------------------------------------------

        _moduleLocation = config.position;
        _moduleIndex = config.index;
        _desiredModuleState = new SwerveModuleState(0.0, new Rotation2d(0.0));
    }

    @Override
    public void periodic() {
        updateState();
    }

    // ======================================================================================

    public void setState(SwerveModuleState state) {
        SwerveModuleState optimizedState = new SwerveModuleState(
            state.speedMetersPerSecond, 
            state.angle
        );
        optimizedState.optimize(getSteerAngle());
        
        _desiredModuleState.speedMetersPerSecond = optimizedState.speedMetersPerSecond;
        _desiredModuleState.angle = optimizedState.angle;
    }

    public void setState(double speedMetersPerSecond, Rotation2d angle) {
        setState(new SwerveModuleState(speedMetersPerSecond, angle));
    }

    private void updateState() {
        _steerMotor.set(_steerPositionSpeed.calculate(_encoder.getAbsolutePosition().getValueAsDouble(), _desiredModuleState.angle.getRotations()));

        double driveRotationsPerSecond = _desiredModuleState.speedMetersPerSecond 
            / (Constants.Drivetrain.Hardware.WHEEL_DIAMETER_METER * Math.PI);

        _driveMotor.setControl(_driveVelocityVoltage.withVelocity(driveRotationsPerSecond));
    }

    // ======================================================================================

    public Rotation2d getSteerAngle() {
        return Rotation2d.fromRotations(_encoder.getAbsolutePosition().getValueAsDouble());
    }

    public double getDriveVelocityMetersPerSecond() {
        double rps = _driveMotor.getVelocity().getValueAsDouble();
        return rps * Constants.Drivetrain.Hardware.WHEEL_DIAMETER_METER * Math.PI;
    }

    public SwerveModuleState getDesiredState() {
        return _desiredModuleState;
    }

    public double getDrivePositionMeters() {
         double rotations = _driveMotor.getPosition().getValueAsDouble();
         return rotations * Constants.Drivetrain.Hardware.WHEEL_DIAMETER_METER * Math.PI;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            getDrivePositionMeters(),
            getSteerAngle()
        );
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            getDriveVelocityMetersPerSecond(),
            getSteerAngle()
        );
    }

    public boolean isHealthy() {
        return _steerMotor.isAlive() && _driveMotor.isAlive() && _encoder.isConnected();
    }

    public Translation2d getLocation() {
        return _moduleLocation;
    }

    public int getIndex() {
        return _moduleIndex;
    }
    
    // ======================================================================================

    private StatusCode applyConfigWithRetry(String motorRole, String moduleName, java.util.concurrent.Callable<StatusCode> applier) {
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

    public static class ModuleConfiguration {
        public String moduleName = "";
        public int index = 0;
        public Translation2d position = new Translation2d();
        public Rotation2d encoderOffset = new Rotation2d(0.0);
    }
}
