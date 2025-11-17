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
import frc.robot.commands.ZeroIMU;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
	public static final CommandXboxController driver = new CommandXboxController(0);
	public static final CommandXboxController operator = new CommandXboxController(1);

	private final SendableChooser<Command> autoChooser;

    Drivetrain _drivetrain;

	public RobotContainer() {
        _drivetrain = new Drivetrain();

		autoChooser = AutoBuilder.buildAutoChooser();

    	SmartDashboard.putData("Auto Chooser", autoChooser);

		configureBindings();
	}

	private void configureBindings() {
		_drivetrain.setDefaultCommand(new Drive(_drivetrain));

		driver.b().onTrue(new ZeroIMU(_drivetrain));
		driver.a().onTrue(new Snap(_drivetrain, Rotation2d.fromDegrees(0.0)));
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
