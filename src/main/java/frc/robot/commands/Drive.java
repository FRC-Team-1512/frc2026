package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SuperStructure;

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
        double distance = Constants.TARGET.getDistance(currentPose.getTranslation());
        Rotation2d angleToTarget = Constants.TARGET.minus(currentPose.getTranslation()).getAngle();

        _superStructure.setShootDistance(distance);
        boolean isSlowMode = RobotContainer.driver.leftBumper().getAsBoolean();

        double vx = -applyDeadband(RobotContainer.driver.getLeftY(), 0.15);
        double vy = -applyDeadband(RobotContainer.driver.getLeftX(), 0.15);

        double rot = -applyDeadband(RobotContainer.driver.getRightX(), 0.15);

        if(!isSlowMode) {
            vx *= 3.0;
            vy *= 3.0;
            rot *= 3.0;
        }
        
        if (_superStructure.isShootingMode()) {
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
