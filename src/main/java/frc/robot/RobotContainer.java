// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Drive;
import frc.robot.commands.Snap;
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
import frc.robot.subsystems.SuperStructure;

public class RobotContainer {
	public static final CommandXboxController driver = new CommandXboxController(0);
	public static final CommandXboxController operator = new CommandXboxController(1);

	private final SendableChooser<Command> autoChooser;

    Drivetrain _drivetrain;
    //Shooter _shooter;
	//Indexer _indexer;
	//Intake _intake;
	SuperStructure _superStructure;
	public RobotContainer() {
        _drivetrain = new Drivetrain();
		//_shooter = new Shooter();
		//_indexer = new Indexer();
		//_intake = new Intake();
		_superStructure = new SuperStructure(new Intake(), new Indexer(), new Shooter(), _drivetrain::getPose, _drivetrain::getVelocity);
		autoChooser = AutoBuilder.buildAutoChooser();

    	SmartDashboard.putData("Auto Chooser", autoChooser);

		configureBindings();
	}

	private void configureBindings() {
		_drivetrain.setDefaultCommand(new Drive(_drivetrain, _superStructure));
		//_shooter.setDefaultCommand(new ShooterTest(_shooter));
		//_indexer.setDefaultCommand(new IndexerTest(_indexer));
		//_intake.setDefaultCommand(new IntakeTest(_intake));

		driver.y().onTrue(new Snap(_drivetrain, Rotation2d.fromDegrees(180.0)));
		driver.x().onTrue(new Snap(_drivetrain, Rotation2d.fromDegrees(-45.0)));

		driver.rightTrigger().onTrue(_superStructure.requestShoot());
		driver.rightTrigger().onFalse(_superStructure.revokeShoot());
		driver.leftTrigger().onTrue(_superStructure.requestIntake());
		driver.leftTrigger().onFalse(_superStructure.revokeIntake());
		driver.b().onTrue(_superStructure.requestIdleExpanded());
		driver.a().onTrue(_superStructure.requestIdle());

		driver.leftBumper().onTrue(_superStructure.isManual());
		driver.leftBumper().onFalse(_superStructure.isNotManual());

		NamedCommands.registerCommand("requestShoot", _superStructure.requestShoot());
		NamedCommands.registerCommand("requestIntake", _superStructure.requestIntake());
		NamedCommands.registerCommand("requestIdle", _superStructure.requestIdle());
		NamedCommands.registerCommand("requestIdleExpanded", _superStructure.requestIdleExpanded());
		NamedCommands.registerCommand("revokeShoot", _superStructure.revokeShoot());
		NamedCommands.registerCommand("revokeIntake", _superStructure.revokeIntake());
		NamedCommands.registerCommand("isManual", _superStructure.isManual());
		NamedCommands.registerCommand("isNotManual", _superStructure.isNotManual());

		//operator.a().onTrue(new DecreaseFlyWheel(_shooter));
		//operator.y().onTrue(new IncreaseFlyWheel(_shooter));
		//operator.x().onTrue(new DecreaseHood(_shooter));
		//operator.b().onTrue(new IncreaseHood(_shooter));

	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
