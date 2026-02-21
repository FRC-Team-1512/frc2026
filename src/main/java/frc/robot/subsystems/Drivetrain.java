package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.utils.LimelightHelpers;


public class Drivetrain extends SubsystemBase {
    public static final int FL_IDX = 0;
    public static final int FR_IDX = 1;
    public static final int BL_IDX = 2;
    public static final int BR_IDX = 3;

    private SwerveModule[] _modules;
    private final Pigeon2 _gyro;

    private ChassisSpeeds _desiredChassisSpeeds;
    private ChassisSpeeds _previousDesiredChassisSpeeds;
    private SwerveModulePosition[] _measuredPositions;

    private Rotation2d _headingTarget;
    private final PIDController _headingController;

    private Pose2d _currentPose;
    private Pose2d _previousPose;

    private final SwerveDriveKinematics _kinematics;
    private final SwerveDrivePoseEstimator _odometry;

    private final boolean _isRedAlliance;

    private double _lastTime;
    private double _deltaT;

    DoublePublisher _timePublisher;
    StructArrayPublisher<Rotation2d> _headingPublisher;
    StructArrayPublisher<SwerveModuleState> _desiredSwerveStatePublisher;
    StructArrayPublisher<Pose2d> _currentPosePublisher;
    StructArrayPublisher<Pose2d> _visionPosePublisher;
    StructArrayPublisher<Pose2d> _mt2PosePublisher;
    StructArrayPublisher<Rotation2d> _visionHeading;
    StructArrayPublisher<Rotation2d> _desiredHeadingPublisher;
    IntegerPublisher _visionFiducialIDPublisher;
    StructArrayPublisher<Pose2d> _limelightLeftPosePublisher;
    StructArrayPublisher<Pose2d> _limelightRightPosePublisher;
<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream

    private boolean _isSeed;
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes

    public Drivetrain() {
        _modules = new SwerveModule[4];
        _modules[FL_IDX] = new SwerveModule(RobotMap.CAN.FL_STEER, RobotMap.CAN.FL_DRIVE, RobotMap.CAN.FL_ENCODER,
                Constants.Drivetrain.FL_CONFIG);
        _modules[FR_IDX] = new SwerveModule(RobotMap.CAN.FR_STEER, RobotMap.CAN.FR_DRIVE, RobotMap.CAN.FR_ENCODER,
                Constants.Drivetrain.FR_CONFIG);
        _modules[BL_IDX] = new SwerveModule(RobotMap.CAN.BL_STEER, RobotMap.CAN.BL_DRIVE, RobotMap.CAN.BL_ENCODER,
                Constants.Drivetrain.BL_CONFIG);
        _modules[BR_IDX] = new SwerveModule(RobotMap.CAN.BR_STEER, RobotMap.CAN.BR_DRIVE, RobotMap.CAN.BR_ENCODER,
                Constants.Drivetrain.BR_CONFIG);
        _gyro = new Pigeon2(RobotMap.CAN.PIGEON);

        // -------------------------------------------------------------------------------------

        _desiredChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        _previousDesiredChassisSpeeds = _desiredChassisSpeeds;
        _measuredPositions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            _measuredPositions[i] = _modules[i].getPosition();
        }

        _headingTarget = new Rotation2d(0.0);
        _headingController = new PIDController(Constants.Drivetrain.HEADING_KP, Constants.Drivetrain.HEADING_KI,
                Constants.Drivetrain.HEADING_KD);
        _headingController.enableContinuousInput(-Math.PI, Math.PI);

        _currentPose = new Pose2d();
        _previousPose = new Pose2d();

        _kinematics = new SwerveDriveKinematics(
                _modules[FL_IDX].getLocation(),
                _modules[FR_IDX].getLocation(),
                _modules[BL_IDX].getLocation(),
                _modules[BR_IDX].getLocation());

        _odometry = new SwerveDrivePoseEstimator(_kinematics, getHeading(), _measuredPositions, _currentPose);

        // -------------------------------------------------------------------------------------

        Optional<Alliance> alliance = DriverStation.getAlliance();
        _isRedAlliance = alliance.filter(value -> value == Alliance.Red).isPresent();

        _isSeed = false;

        _lastTime = Timer.getFPGATimestamp();
        _deltaT = 1.0 / Constants.TICK_PER_SECOND;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("drivetrain");

        _desiredSwerveStatePublisher = table.getStructArrayTopic("DesiredSwerveStates", SwerveModuleState.struct)
                .publish();
        _currentPosePublisher = table.getStructArrayTopic("CurrentPose", Pose2d.struct).publish();
        _headingPublisher = table.getStructArrayTopic("Heading", Rotation2d.struct).publish();
        _desiredHeadingPublisher = table.getStructArrayTopic("Desired Heading", Rotation2d.struct).publish();
        _timePublisher = table.getDoubleTopic("DeltaTime").publish();
        _visionPosePublisher = table.getStructArrayTopic("VisionPose", Pose2d.struct).publish();
        _mt2PosePublisher = table.getStructArrayTopic("mt2 Pose", Pose2d.struct).publish();
        _visionHeading = table.getStructArrayTopic("vision heading", Rotation2d.struct).publish();
        _visionFiducialIDPublisher = table.getIntegerTopic("vision fiducial ID").publish();
        _limelightLeftPosePublisher = table.getStructArrayTopic("Limelight Left Pose", Pose2d.struct).publish();
        _limelightRightPosePublisher = table.getStructArrayTopic("Limelight Right Pose", Pose2d.struct).publish();

        // -------------------------------------------------------------------------------------

        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
                this::getPose,
                this::resetPose,
                this::getMeasuredRelativeSpeeds,
                (speeds, feedforwards) -> setRobotRelativeSpeeds(speeds),
                new PPHolonomicDriveController(
                        new PIDConstants(Constants.Drivetrain.Auto.TRANSLATION_KP,
                                Constants.Drivetrain.Auto.TRANSLATION_KI,
                                Constants.Drivetrain.Auto.TRANSLATION_KD), // Translation PID constants
                        new PIDConstants(Constants.Drivetrain.Auto.ROTATION_KP,
                                Constants.Drivetrain.Auto.ROTATION_KI,
                                Constants.Drivetrain.Auto.ROTATION_KD) // Rotation PID constants
                ),
                config,
                () -> _isRedAlliance,
                this);

        // -------------------------------------------------------------------------------------

        _isSeed = false;
    }

    @Override
    public void periodic() {
        updateTime();
        updateSpeeds(_desiredChassisSpeeds);
        updateOdometry();
        updateVision();

        _headingPublisher.set(new Rotation2d[] { getHeading() });
        _timePublisher.set(getDeltaT());
        _desiredHeadingPublisher.set(new Rotation2d[] { _headingTarget });
    }

<<<<<<< Updated upstream
    // =======================================================================================
    // Vision
    // =======================================================================================
=======
        // Publish raw Limelight poses (MT1, wpiBlue)
        Pose2d leftPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-left");
        Pose2d rightPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-right");
        _limelightLeftPosePublisher.set(new Pose2d[] { leftPose });
        _limelightRightPosePublisher.set(new Pose2d[] { rightPose });

        SmartDashboard.putNumber("LL Left X", leftPose.getX());
        SmartDashboard.putNumber("LL Left Y", leftPose.getY());
        SmartDashboard.putNumber("LL Right X", rightPose.getX());
        SmartDashboard.putNumber("LL Right Y", rightPose.getY());
        SmartDashboard.putNumber("Pigeon Degrees", getHeading().getDegrees());

        // Publish raw Limelight poses (MT1, wpiBlue)
        Pose2d leftPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-left");
        Pose2d rightPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-right");
        _limelightLeftPosePublisher.set(new Pose2d[] { leftPose });
        _limelightRightPosePublisher.set(new Pose2d[] { rightPose });

        SmartDashboard.putNumber("LL Left X", leftPose.getX());
        SmartDashboard.putNumber("LL Left Y", leftPose.getY());
        SmartDashboard.putNumber("LL Right X", rightPose.getX());
        SmartDashboard.putNumber("LL Right Y", rightPose.getY());
        SmartDashboard.putNumber("Pigeon Degrees", getHeading().getDegrees());

        // Publish raw Limelight poses (MT1, wpiBlue)
        Pose2d leftPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-left");
        Pose2d rightPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-right");
        _limelightLeftPosePublisher.set(new Pose2d[] { leftPose });
        _limelightRightPosePublisher.set(new Pose2d[] { rightPose });

        SmartDashboard.putNumber("LL Left X", leftPose.getX());
        SmartDashboard.putNumber("LL Left Y", leftPose.getY());
        SmartDashboard.putNumber("LL Right X", rightPose.getX());
        SmartDashboard.putNumber("LL Right Y", rightPose.getY());
        SmartDashboard.putNumber("Pigeon Degrees", getHeading().getDegrees());

        // Publish raw Limelight poses (MT1, wpiBlue)
        Pose2d leftPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-left");
        Pose2d rightPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-right");
        _limelightLeftPosePublisher.set(new Pose2d[] { leftPose });
        _limelightRightPosePublisher.set(new Pose2d[] { rightPose });

        SmartDashboard.putNumber("LL Left X", leftPose.getX());
        SmartDashboard.putNumber("LL Left Y", leftPose.getY());
        SmartDashboard.putNumber("LL Right X", rightPose.getX());
        SmartDashboard.putNumber("LL Right Y", rightPose.getY());
        SmartDashboard.putNumber("Pigeon Degrees", getHeading().getDegrees());

        /*
        if(_isSeed) {
            publishBestId();
        }*/
>>>>>>> Stashed changes

    /**
     * Sets the Limelight IMU mode for all configured Limelights.
     * Mode 1 = EXTERNAL_SEED (use during Disabled to keep seeding Pigeon2 yaw)
     * Mode 4 = INTERNAL_EXTERNAL_ASSIST (use during Enabled for 1kHz fusion)
     */
    public void setLimelightIMUMode(int mode) {
        for (String name : Constants.Drivetrain.Vision.LIMELIGHT_NAMES) {
            LimelightHelpers.SetIMUMode(name, mode);
        }
    }

    /**
     * Forces a re-seed of the gyro heading from MT1 on the next periodic cycle.
     * Call this from Disabled or from a command to re-calibrate heading.
     */
    public void forceReseed() {
        _isSeed = false;
    }

    public boolean isSeed() {
        return _isSeed;
    }

    private void updateVision() {
        // Manage IMU mode: Disabled -> mode 1 (external seed), Enabled -> mode 4 (fusion)
        if (DriverStation.isDisabled()) {
            setLimelightIMUMode(1);
        } else {
            setLimelightIMUMode(4);
        }

        for (String limelightName : Constants.Drivetrain.Vision.LIMELIGHT_NAMES) {
            if (!_isSeed) {
                // Phase 1: MT1 seed — use vision-only heading to calibrate the gyro
                attemptMT1Seed(limelightName);
            } else {
                // Phase 2: MT2 continuous pose estimation
                updateMT2(limelightName);
            }
        }
    }

    /**
     * Phase 1: Attempts to seed the Pigeon2 heading from a Limelight MT1 pose.
     * MT1 computes heading purely from the image, so it does not depend on the gyro.
     * Once a reliable reading is obtained, the gyro yaw is set and _isSeed flips to true.
     */
    private void attemptMT1Seed(String limelightName) {
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        if (mt1 == null || mt1.tagCount == 0) {
            return;
        }

        // Check ambiguity — reject if any tag is too ambiguous
        for (LimelightHelpers.RawFiducial fiducial : mt1.rawFiducials) {
            if (fiducial.ambiguity > Constants.Drivetrain.Vision.MT1_AMBIGUITY_THRESHOLD) {
                return;
            }
        }

        // MT1 heading is trustworthy — seed the Pigeon2
        Rotation2d visionHeading = mt1.pose.getRotation();
        _gyro.setYaw(visionHeading.getDegrees());
        _headingTarget = visionHeading;
        _isSeed = true;

        _visionHeading.set(new Rotation2d[] { visionHeading });
        _visionPosePublisher.set(new Pose2d[] { mt1.pose });
    }

    /**
     * Phase 2: Feeds the current Pigeon2 yaw into Limelight, then reads back
     * the MegaTag2 pose estimate and fuses it into the WPILib pose estimator.
     */
    private void updateMT2(String limelightName) {
        // Tell Limelight our current heading every frame (required for MT2)
        double yawDeg = getHeading().getDegrees();
        double yawRateDps = _gyro.getAngularVelocityZWorld().getValueAsDouble();
        LimelightHelpers.SetRobotOrientation(limelightName,
                yawDeg, yawRateDps,
                0.0, 0.0,
                0.0, 0.0);

        // Reject if spinning too fast — MT2 becomes unreliable
        if (Math.abs(yawRateDps) > Constants.Drivetrain.Vision.MAX_ANGULAR_VELOCITY_DEG_PER_SEC) {
            return;
        }

        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (mt2 == null || mt2.tagCount == 0) {
            return;
        }

        _odometry.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);

        _mt2PosePublisher.set(new Pose2d[] { mt2.pose });
    }

    // =======================================================================================

    private void updateSpeeds(ChassisSpeeds speeds) {
        if (speeds == null) {
            speeds = new ChassisSpeeds();
        }

        ChassisSpeeds limited = limitChassisAcceleration(
                speeds,
                _previousDesiredChassisSpeeds,
                getDeltaT(),
                Constants.Drivetrain.MAX_TRANSLATOIN_ACCELERATION_METERS_PER_SECOND_SQUARED,
                Constants.Drivetrain.MAX_ROTATION_ACCELERATION_RADIANS_PER_SECOND_SQUARED);

        SwerveModuleState desiredStates[] = _kinematics.toSwerveModuleStates(limited);
        setModuleStates(desiredStates);

        _previousDesiredChassisSpeeds = limited;
    }

    public void setRobotRelativeSpeeds(ChassisSpeeds speeds) {
        _desiredChassisSpeeds = speeds;
    }

    public void setRobotRelativeSpeeds(double vx, double vy, double omega) {
        setRobotRelativeSpeeds(new ChassisSpeeds(vx, vy, omega));
    }

    public void setFieldRelativeSpeeds(ChassisSpeeds fieldRelativeSpeeds) {
        _headingTarget = _headingTarget
                .plus(Rotation2d.fromRadians(fieldRelativeSpeeds.omegaRadiansPerSecond * getDeltaT()));
        double headingCorrectionOmegaRadiansPerSecond = _headingController.calculate(getHeading().getRadians(),
                _headingTarget.getRadians()) * Constants.Drivetrain.HEADING_COEFF;
        fieldRelativeSpeeds.omegaRadiansPerSecond += headingCorrectionOmegaRadiansPerSecond;

        ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                fieldRelativeSpeeds,
                getHeading());
        setRobotRelativeSpeeds(robotRelativeSpeeds);
    }

    public void setFieldRelativeSpeeds(double vx, double vy, double omega) {
        setFieldRelativeSpeeds(new ChassisSpeeds(vx, vy, omega));
    }

    private void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND);
        for (SwerveModule module : _modules) {
            module.setState(states[module.getIndex()]);
        }
        _desiredSwerveStatePublisher.set(states);
    }

    public void stop() {
        setRobotRelativeSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    public ChassisSpeeds getMeasuredRelativeSpeeds() {
        if (_modules == null || _modules.length == 0) {
            return new ChassisSpeeds();
        }
        SwerveModuleState[] states = new SwerveModuleState[_modules.length];
        for (SwerveModule module : _modules) {
            states[module.getIndex()] = module.getState();
        }
        return _kinematics.toChassisSpeeds(states);
    }

    private ChassisSpeeds limitChassisAcceleration(ChassisSpeeds desired, ChassisSpeeds previous, double dt,
            double maxAccel, double maxAngularAccel) {
        double dx = desired.vxMetersPerSecond - previous.vxMetersPerSecond;
        double dy = desired.vyMetersPerSecond - previous.vyMetersPerSecond;
        double dw = desired.omegaRadiansPerSecond - previous.omegaRadiansPerSecond;

        double deltaNorm = Math.sqrt(dx * dx + dy * dy);
        if (deltaNorm > maxAccel * dt) {
            double scale = maxAccel * dt / deltaNorm;
            dx *= scale;
            dy *= scale;
        }

        if (Math.abs(dw) > maxAngularAccel * dt) {
            double scale = maxAngularAccel * dt / Math.abs(dw);
            dw *= scale;
        }

        double vx = previous.vxMetersPerSecond + dx;
        double vy = previous.vyMetersPerSecond + dy;
        double omega = previous.omegaRadiansPerSecond + dw;

        return new ChassisSpeeds(vx, vy, omega);
    }

    public void setHeadingTarget(Rotation2d targetRotation) {
        _headingTarget = targetRotation;
    }

    public void setHeadingTargetDegrees(double targetDegrees) {
        _headingTarget = Rotation2d.fromDegrees(targetDegrees);
    }

    // =======================================================================================

    public void zeroIMU() {
        setIMU(new Rotation2d(0.0));
        _headingTarget = new Rotation2d(0.0);
    }

    public void setIMU(Rotation2d heading) {
        _gyro.setYaw(heading.getDegrees());
        _headingTarget = heading;
    }

    public Rotation2d getHeading() {
        Rotation2d heading = _gyro.getRotation2d();
        if (Constants.Drivetrain.PIGEON_INVERTED) {
            heading = heading.unaryMinus();
        }
        return heading;
    }

    // =======================================================================================

    public void updateOdometry() {
        for (SwerveModule module : _modules) {
            _measuredPositions[module.getIndex()] = module.getPosition();
        }

        _odometry.updateWithTime(Timer.getFPGATimestamp(), getHeading(), _measuredPositions);

        _previousPose = _currentPose;
        _currentPose = _odometry.getEstimatedPosition();

        _currentPosePublisher.set(new Pose2d[] { _currentPose });
    }

    public double getVelocityMagnitude() {
        return (_currentPose.getTranslation().getDistance(_previousPose.getTranslation())) * (1.0 / getDeltaT());
    }

    public Pose2d getPose() {
        return _currentPose;
    }

    public void resetPose(Pose2d pose) {
        _odometry.resetPose(pose);
        _currentPose = pose;
        _previousPose = pose;
        _lastTime = Timer.getFPGATimestamp();
        _headingTarget = pose.getRotation();
    }

    private void updateTime() {
        double currentTime = Timer.getFPGATimestamp();
        _deltaT = currentTime - _lastTime;
        _lastTime = currentTime;
    }

    private double getDeltaT() {
        return _deltaT;
    }
}
