// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Drive;
import frc.robot.commands.Snap;
import frc.robot.commands.SetIMU;
import frc.robot.commands.ZeroIMU;
import frc.robot.commands.test.DecreaseFlyWheel;
import frc.robot.commands.test.DecreaseHood;

import frc.robot.commands.test.IncreaseFlyWheel;
import frc.robot.commands.test.IncreaseHood;

import frc.robot.commands.test.IndexerTest;
import frc.robot.commands.test.IntakeTest;
import frc.robot.commands.test.ShooterTest;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class RobotContainer {
	public static final CommandXboxController driver = new CommandXboxController(0);
	public static final CommandXboxController operator = new CommandXboxController(1);

	private final SendableChooser<Command> autoChooser;

    Drivetrain _drivetrain;
    Shooter _shooter;
	Indexer _indexer;
	Intake _intake;
	public RobotContainer() {
        _drivetrain = new Drivetrain();
		_shooter = new Shooter();
		_indexer = new Indexer();
		_intake = new Intake();
		autoChooser = AutoBuilder.buildAutoChooser();

    	SmartDashboard.putData("Auto Chooser", autoChooser);

		configureBindings();
	}

	private void configureBindings() {
		_drivetrain.setDefaultCommand(new Drive(_drivetrain));
		_shooter.setDefaultCommand(new ShooterTest(_shooter));
		_indexer.setDefaultCommand(new IndexerTest(_indexer));
		_intake.setDefaultCommand(new IntakeTest(_intake));

		driver.b().onTrue(new SetIMU(_drivetrain, Rotation2d.fromDegrees(45)));
		driver.y().onTrue(new Snap(_drivetrain, Rotation2d.fromDegrees(180.0)));
		driver.x().onTrue(new Snap(_drivetrain, Rotation2d.fromDegrees(-45.0)));
		driver.a().onTrue(new ZeroIMU(_drivetrain));

		operator.a().onTrue(new DecreaseFlyWheel(_shooter));
		operator.y().onTrue(new IncreaseFlyWheel(_shooter));
		operator.x().onTrue(new DecreaseHood(_shooter));
		operator.b().onTrue(new IncreaseHood(_shooter));
		operator.rightBumper().onTrue(_intake.runIntakeWheel(0.8));
		operator.rightBumper().onFalse(_intake.runIntakeWheel(0.0));
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
