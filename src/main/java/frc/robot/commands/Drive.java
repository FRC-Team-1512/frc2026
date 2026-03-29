package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotation;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    
    public Drive(Drivetrain drivetrain, SuperStructure superStructure) {
        _drivetrain = drivetrain;
        _superStructure = superStructure;
        addRequirements(_drivetrain, _superStructure);
    }
    
    @Override
    public void execute() {
        Pose2d currentPose = _drivetrain.getPose();
        
        Optional<Alliance> alliance = DriverStation.getAlliance();
        boolean _isRedAlliance = alliance.filter(value -> value == Alliance.Red).isPresent();

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

        //Translation2d adjustedTarget = target.minus(_drivetrain.getVelocity().times(ShooterCalc.T_ETA));
        Translation2d adjustedTarget = target; // no shoot on move

        Rotation2d angleToTarget = adjustedTarget.minus(currentPose.getTranslation()).getAngle();

        SmartDashboard.putNumber("Drive: angleToTarget", angleToTarget.getDegrees());

        boolean isSlowMode = RobotContainer.driver.leftBumper().getAsBoolean();

        double vx = -applyDeadband(RobotContainer.driver.getLeftY(), 0.15);
        double vy = -applyDeadband(RobotContainer.driver.getLeftX(), 0.15);

        double rot = -applyDeadband(RobotContainer.driver.getRightX(), 0.15);

        double translationCoeff = isSlowMode ? 1.2 : 3.2;
        double rotationCoeff = isSlowMode ? 1.2 : 2.0;

        vx *= translationCoeff;
        vy *= translationCoeff;
        rot *= rotationCoeff;

        vx *= invert;
        vy *= invert;
        
        if (_superStructure.isShootingMode() && !RobotContainer.driver.rightBumper().getAsBoolean()) {
            SmartDashboard.putNumber("angleToTarget", angleToTarget.getDegrees());
            _drivetrain.setFieldRelativeSpeedsWithHeading(vx, vy, angleToTarget);
        } else {
            _drivetrain.setFieldRelativeSpeeds(new ChassisSpeeds(vx, vy, rot));
        }

        SmartDashboard.putNumber("vx", vx);
        SmartDashboard.putNumber("vy", vy);
        SmartDashboard.putNumber("rot", rot);
    }

    private static double applyDeadband(double input, double deadband) {
        if (Math.abs(input) <= deadband) {
            return 0.0;
        }else {
            return input;
        }
    }
}
