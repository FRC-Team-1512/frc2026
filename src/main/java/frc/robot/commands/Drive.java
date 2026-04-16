package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotation;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SuperStructure;
import frc.robot.utils.ShooterCalc;

public class Drive extends Command {
    Drivetrain _drivetrain;
    SuperStructure _superStructure;

    SlewRateLimiter _vxLimiter = new SlewRateLimiter(16.0);
    SlewRateLimiter _vyLimiter = new SlewRateLimiter(16.0);
    SlewRateLimiter _rotLimiter = new SlewRateLimiter(10.0);
    
    public Drive(Drivetrain drivetrain, SuperStructure superStructure) {
        _drivetrain = drivetrain;
        _superStructure = superStructure;
        addRequirements(_drivetrain, _superStructure);
    }
    
    @Override
    public void execute() {
        Pose2d currentPose = _drivetrain.getPose();
        
        boolean _isRedAlliance = RobotContainer.isRedAlliance();

        int invert = 1;
        if(_isRedAlliance) {
            invert = -1;
        }     
        Translation2d target;
        if(_isRedAlliance) {
            target = Constants.TARGET_RED;
        } else {
            target = Constants.TARGET_BLUE;
        }

        Translation2d adjustedTarget = target.minus(_drivetrain.getDeadbandedVelocity(0.5).times(ShooterCalc.T_ETA));
        //Translation2d adjustedTarget = target; // when no shoot on move

        Rotation2d angleToTarget = adjustedTarget.minus(currentPose.getTranslation()).getAngle();

        boolean isSlowMode = RobotContainer.driver.leftBumper().getAsBoolean();

        double vx = -MathUtil.applyDeadband(RobotContainer.driver.getLeftY(), 0.15);
        double vy = -MathUtil.applyDeadband(RobotContainer.driver.getLeftX(), 0.15);
        double rot = -MathUtil.applyDeadband(RobotContainer.driver.getRightX(), 0.15);

        vx = Math.copySign(vx * vx, vx);
        vy = Math.copySign(vy * vy, vy);
        rot = Math.copySign(rot * rot, rot);

        double translationCoeff = isSlowMode ? 1.2 : 3.67;
        double rotationCoeff = isSlowMode ? 1.2 : 2.0;

        vx *= translationCoeff;
        vy *= translationCoeff;
        rot *= rotationCoeff;

        vx *= invert;
        vy *= invert;

        vx = _vxLimiter.calculate(vx);
        vy = _vyLimiter.calculate(vy);
        rot = _rotLimiter.calculate(rot);

        if (_superStructure.isShootingMode() && !RobotContainer.driver.leftTrigger().getAsBoolean()) {
            SmartDashboard.putNumber("angleToTarget", angleToTarget.getDegrees());
            _drivetrain.setFieldRelativeSpeedsWithHeading(vx, vy, angleToTarget);
        } else {
            _drivetrain.setFieldRelativeSpeeds(new ChassisSpeeds(vx, vy, rot));
        }

        SmartDashboard.putNumber("vx", vx);
        SmartDashboard.putNumber("vy", vy);
        SmartDashboard.putNumber("rot", rot);
    }
}
