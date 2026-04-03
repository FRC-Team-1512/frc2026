package frc.robot.subsystems;

import java.util.EnumSet;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.ShooterCalc;

public class SuperStructure extends SubsystemBase {
    private final Intake _intake;
    private final Indexer _indexer;
    private final Shooter _shooter;
    private final Supplier<Pose2d> _poseSupplier;
    private final Supplier<Translation2d> _velocitySupplier;

    private boolean _isManual = false;

    private double _targetDistanceMeters;

    private final EnumSet<SuperStructureState> _activeStates = EnumSet.noneOf(SuperStructureState.class);
    private SuperStructureState _defaultIdleState;

    public final DirectControl direct = new DirectControl();

    public SuperStructure(Intake intake, Indexer indexer, Shooter shooter, Supplier<Pose2d> poseSupplier, Supplier<Translation2d> velocitySupplier) {
        _intake = intake;
        _indexer = indexer;
        _shooter = shooter;
        _poseSupplier = poseSupplier;
        _velocitySupplier = velocitySupplier;

        _targetDistanceMeters = 0.0;

        _defaultIdleState = SuperStructureState.IDLE;

        _activeStates.add(_defaultIdleState);
    }

    @Override
    public void periodic() {

        if (DriverStation.isDisabled()) {
            requestState(_defaultIdleState);
            setIsManual(false);
        }

        Pose2d currentPose = _poseSupplier.get();

        boolean _isRedAlliance = RobotContainer.isRedAlliance();

        Translation2d target;
        if(_isRedAlliance) {
            target = Constants.TARGET_RED;
        } else {
            target = Constants.TARGET_BLUE;
        }

        Translation2d adjustedTarget = target.minus(_velocitySupplier.get().times(ShooterCalc.T_ETA));
        //Translation2d adjustedTarget = target; // when no shoot on move

        if(_isManual) {
            _targetDistanceMeters = Constants.DEFAULT_DISTANCE;
        } else {
            _targetDistanceMeters = adjustedTarget.getDistance(currentPose.getTranslation());
        }

        SmartDashboard.putNumber("SuperStructure: Distance Meters", _targetDistanceMeters);

        if (_activeStates.contains(SuperStructureState.INTAKE)) {
            _intake.intake();
        }
        if (_activeStates.contains(SuperStructureState.REVERSE_INTAKE)) {
            _intake.reverseIntake();
        }
        if (!_activeStates.contains(SuperStructureState.REVERSE_INTAKE) && !_activeStates.contains(SuperStructureState.INTAKE) && _defaultIdleState == SuperStructureState.IDLE) {
            _intake.retract();
        }
        if (_activeStates.contains(SuperStructureState.SHOOT)) {
            double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            // 0.6 sec period, Robot Mormonism

            boolean isExtended = (currentTime % 0.6) < 0.3;
            /*
            if (isExtended) {
                _intake.extendArm();
            } else {
                // _intake.extendHalfArm(); // position mode is disabled
                _intake.retractArm();
            }
                */

            _shooter.setShooterFromDistance(_targetDistanceMeters);
            if (_shooter.isReadyToShoot()) {
                _indexer.setIndexer(1.8);
            } else {
                _indexer.setIndexer(0.0);
            }
        } else if (!_activeStates.contains(SuperStructureState.IDLE) && !_activeStates.contains(SuperStructureState.IDLE_EXPANDED)) {
            _shooter.setShooterVelocity(0.0);
            _indexer.setIndexer(0.0);
        }
        if (_activeStates.contains(SuperStructureState.IDLE)) {
            _intake.retract();
            _indexer.setIndexer(0.0);
            _shooter.setShooterVelocity(0.0);
            _shooter.setHoodRest();
        }
        if (_activeStates.contains(SuperStructureState.IDLE_EXPANDED)) {
            _intake.extendArm();
            _intake.setIntakeWheel(0.0);
            _indexer.setIndexer(0.0);
            _shooter.setShooterVelocity(0.0);
            _shooter.setHoodRest();
        }

        publishState();
    }

    public void publishState() {
        SmartDashboard.putBoolean("SuperStructure: IDLE", _activeStates.contains(SuperStructureState.IDLE));
        SmartDashboard.putBoolean("SuperStructure: IDLE EXPANDED", _activeStates.contains(SuperStructureState.IDLE_EXPANDED));
        SmartDashboard.putBoolean("SuperStructure: INTAKE", _activeStates.contains(SuperStructureState.INTAKE));
        SmartDashboard.putBoolean("SuperStructure: REVERSE_INTAKE", _activeStates.contains(SuperStructureState.REVERSE_INTAKE));
        SmartDashboard.putBoolean("SuperStructure: SHOOT", _activeStates.contains(SuperStructureState.SHOOT));
    }

    public void requestState(SuperStructureState state) {
        sanitizeStates(state);
    }

    public void revokeState(SuperStructureState state) {
        _activeStates.remove(state);
        sanitizeStates();
    }

    public void setIsManual(boolean isManual) {
        _isManual = isManual;
    }

    // public void setShootDistance(double distanceMeters) {
    //     _targetDistanceMeters = distanceMeters;
    // }

    public boolean isShootingMode() {
        return _activeStates.contains(SuperStructureState.SHOOT);
    }

    public class DirectControl {
        public void setIntakeWheel(double power) {
            _intake.setIntakeWheel(power);
        }

        public void setIntakeArm(Rotation2d position) {
            // _intake.setIntakeArm(position);
        }

        public void setIndexer(double power) {
            _indexer.setIndexer(power);
        }

        public void setShooterVelocity(double velocity) {
            _shooter.setShooterVelocity(velocity);
        }

        public void setShooterHoodAngle(double rotations) {
            _shooter.setHoodAngleRotations(rotations);
        }
    }

    public enum SuperStructureState {
        IDLE,
        IDLE_EXPANDED,
        INTAKE,
        REVERSE_INTAKE,
        SHOOT,
    }

    private void sanitizeStates(SuperStructureState newState) {
        if (newState == SuperStructureState.REVERSE_INTAKE) {
            _activeStates.remove(SuperStructureState.INTAKE);
        } else if (newState == SuperStructureState.INTAKE) {
            _activeStates.remove(SuperStructureState.REVERSE_INTAKE);
        }
        if (newState == SuperStructureState.IDLE) {
            _defaultIdleState = SuperStructureState.IDLE;
            _activeStates.remove(SuperStructureState.IDLE_EXPANDED);
        } else if (newState == SuperStructureState.IDLE_EXPANDED) {
            _defaultIdleState = SuperStructureState.IDLE_EXPANDED;
            _activeStates.remove(SuperStructureState.IDLE);
        }
        if (newState != SuperStructureState.IDLE && newState != SuperStructureState.IDLE_EXPANDED) {
            _activeStates.remove(SuperStructureState.IDLE);
            _activeStates.remove(SuperStructureState.IDLE_EXPANDED);
        }
        _activeStates.add(newState);
        sanitizeStates();
    }

    private void sanitizeStates() {
        if (_activeStates.isEmpty()) {
            _activeStates.add(_defaultIdleState);
        }
        if (_activeStates.contains(SuperStructureState.IDLE) && _activeStates.contains(SuperStructureState.IDLE_EXPANDED)) {
            _activeStates.remove(SuperStructureState.IDLE_EXPANDED);
        }
        if (_activeStates.contains(SuperStructureState.INTAKE) && _activeStates.contains(SuperStructureState.REVERSE_INTAKE)) {
            _activeStates.remove(SuperStructureState.REVERSE_INTAKE);
        }
        if (_activeStates.contains(SuperStructureState.IDLE) || _activeStates.contains(SuperStructureState.IDLE_EXPANDED)) {
            _activeStates.remove(SuperStructureState.INTAKE);
            _activeStates.remove(SuperStructureState.REVERSE_INTAKE);
            _activeStates.remove(SuperStructureState.SHOOT);
        }
    }

    public Command requestIntake() {
        return runOnce(() -> requestState(SuperStructureState.INTAKE));
    }

    public Command requestReverseIntake() {
        return runOnce(() -> requestState(SuperStructureState.REVERSE_INTAKE));
    }

    public Command requestShoot() {
        return runOnce(() -> requestState(SuperStructureState.SHOOT));
    }

    public Command requestIdle() {
        return runOnce(() -> requestState(SuperStructureState.IDLE));
    }

    public Command requestIdleExpanded() {
        return runOnce(() -> requestState(SuperStructureState.IDLE_EXPANDED));
    }

    public Command requestDefaultIdle() {
        return runOnce(() -> requestState(_defaultIdleState));
    }

    public Command revokeShoot() {
        return runOnce(() -> revokeState(SuperStructureState.SHOOT));
    }

    public Command revokeIntake() {
        return runOnce(() -> revokeState(SuperStructureState.INTAKE));
    }

    public Command revokeReverseIntake() {
        return runOnce(() -> revokeState(SuperStructureState.REVERSE_INTAKE));
    }

    public Command isManual() {
        return runOnce(() -> setIsManual(true));
    }

    public Command isNotManual() {
        return runOnce(() -> setIsManual(false));
    }
}
