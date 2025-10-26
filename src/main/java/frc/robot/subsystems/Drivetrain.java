package frc.robot.subsystems;


import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Drivetrain extends SubsystemBase {
    public static final int FL_IDX = 0;
    public static final int FR_IDX = 1;
    public static final int BL_IDX = 2;
    public static final int BR_IDX = 3;

    private SwerveModule[] _modules;
    private final Pigeon2 _gyro;

    private ChassisSpeeds _desiredChassisSpeeds;
    private SwerveModuleState[] _measuredStates;
    private SwerveModulePosition[] _measuredPositions;
    private Rotation2d _yaw;

    private double _yawOffset;

    private Pose2d _currentPose;
    private Pose2d _previousPose;

    private final SwerveDriveKinematics _kinematics;
    private final SwerveDriveOdometry _odometry;

    private final boolean _isRedAlliance;

    public Drivetrain() {
        _modules = new SwerveModule[4];
        _modules[FL_IDX] = new SwerveModule(RobotMap.CAN.FL_STEER, RobotMap.CAN.FL_DRIVE, RobotMap.CAN.FL_ENCODER, Constants.Drivetrain.FL_CONFIG);
        _modules[FR_IDX] = new SwerveModule(RobotMap.CAN.FR_STEER, RobotMap.CAN.FR_DRIVE, RobotMap.CAN.FR_ENCODER, Constants.Drivetrain.FR_CONFIG);
        _modules[BL_IDX] = new SwerveModule(RobotMap.CAN.BL_STEER, RobotMap.CAN.BL_DRIVE, RobotMap.CAN.BL_ENCODER, Constants.Drivetrain.BL_CONFIG);
        _modules[BR_IDX] = new SwerveModule(RobotMap.CAN.BR_STEER, RobotMap.CAN.BR_DRIVE, RobotMap.CAN.BR_ENCODER, Constants.Drivetrain.BR_CONFIG);
        _gyro = new Pigeon2(RobotMap.CAN.PIGEON);

        _desiredChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        _measuredStates = new SwerveModuleState[4];
        _measuredPositions = new SwerveModulePosition[4];
        _yaw = new Rotation2d(0.0);

        _yawOffset = _gyro.getYaw().getValueAsDouble() * Constants.Drivetrain.PIGEON_INVERTED;

        _currentPose = new Pose2d();
        _previousPose = new Pose2d();

        _kinematics = new SwerveDriveKinematics(
            _modules[FL_IDX].getLocation(),
            _modules[FR_IDX].getLocation(),
            _modules[BL_IDX].getLocation(),
            _modules[BR_IDX].getLocation());

        _odometry = new SwerveDriveOdometry(_kinematics, getYaw(), new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        });

        Optional<Alliance> alliance = DriverStation.getAlliance();
        _isRedAlliance = alliance.filter(value -> value == Alliance.Red).isPresent();
    }

    @Override
    public void periodic() {
        setRawSpeeds(_desiredChassisSpeeds);
        updateOdometry();
    }

    private void setRawSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState desiredStates[] = _kinematics.toSwerveModuleStates(speeds);
        for(SwerveModule module : _modules) {
            module.setState(desiredStates[module.getIndex()]);
        }
    }
    
    public void setVelocity(ChassisSpeeds speeds) {
        _desiredChassisSpeeds = speeds;
    }

    public void zeroIMU() {
        _yawOffset = _gyro.getYaw().getValueAsDouble() * Constants.Drivetrain.PIGEON_INVERTED;
        readIMU();
    }

    public void readIMU() {
        double yawRobot = _gyro.getYaw().getValueAsDouble() * Constants.Drivetrain.PIGEON_INVERTED;
        double yawAllianceOffset = _isRedAlliance ? 180.0 : 0.0;
        _yaw = Rotation2d.fromDegrees(yawRobot - _yawOffset + yawAllianceOffset);
    }

    public Rotation2d getYaw() {
        return _yaw;
    }

    public void updateOdometry() {
        for (SwerveModule module : _modules) {
            _measuredStates[module.getIndex()] = module.getDesiredState();
            _measuredPositions[module.getIndex()] = module.getPosition();
        }

        _odometry.update(getYaw(), _measuredPositions);

        _previousPose = _currentPose;
        _currentPose = _odometry.getPoseMeters();
    }

    public double getVelocityMagnitude() {
        return (_currentPose.getTranslation().getDistance(_previousPose.getTranslation())) * Constants.TICK_PER_SECOND;
    }

    public Pose2d getPose() {
        return _currentPose;
    }
}
