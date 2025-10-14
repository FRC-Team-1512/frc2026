package frc.robot.subsystems;


import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;

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
        _modules[FL_IDX] = new SwerveModule(1, 2, 3, Constants.Drivetrain.FL_CONFIG);
        _modules[FR_IDX] = new SwerveModule(4, 5, 6, Constants.Drivetrain.FR_CONFIG);
        _modules[BL_IDX] = new SwerveModule(7, 8, 9, Constants.Drivetrain.BL_CONFIG);
        _modules[BR_IDX] = new SwerveModule(10, 11, 12, Constants.Drivetrain.BR_CONFIG);
        _gyro = new Pigeon2(RobotMap.CAN.PIGEON_CAN);

        _desiredChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        _measuredStates = new SwerveModuleState[4];
        _measuredPositions = new SwerveModulePosition[4];
        _yaw = new Rotation2d(0.0);

        _yawOffset = _gyro.getYaw().getValueAsDouble();

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
        _yawOffset = _gyro.getYaw().getValueAsDouble();
        readIMU();
    }

    public void readIMU() {
        double yawRobot = _gyro.getYaw().getValueAsDouble();
        double yawAllianceOffset = _isRedAlliance ? 180.0 : 0.0;
        _yaw = Rotation2d.fromDegrees(yawRobot - _yawOffset + yawAllianceOffset);
    }

    public Rotation2d getYaw() {
        return _yaw;
    }
}
