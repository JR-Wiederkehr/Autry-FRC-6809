// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

	PneumaticSubsystem m_pneumatic = new PneumaticSubsystem();
  ArmSubsystem m_arm = new ArmSubsystem();


// The driver's controller
//XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
Joystick m_driverController2 = new Joystick(OIConstants.kDriverControllerPort);

/**
 * The container for the robot. Contains subsystems, OI devices, and commands.
 */
public RobotContainer() {
  // Configure the button bindings
  configureButtonBindings();
  // Configure the dashboard
  configureSmartDashboard();

  // Configure default commands
  m_robotDrive.setDefaultCommand(
      // The joystick controls translation and rotation of the robot.
      new RunCommand(
          () -> m_robotDrive.drive(
              -MathUtil.applyDeadband(m_driverController2.getY(), OIConstants.kDriveDeadband), // Y-axis for forward/backward
              -MathUtil.applyDeadband(m_driverController2.getX(), OIConstants.kDriveDeadband), // X-axis for left/right
              -MathUtil.applyDeadband(m_driverController2.getZ(), OIConstants.kDriveDeadband), // Z-axis for rotation
              true),
          m_robotDrive));
}
  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController2, 6)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

       new JoystickButton(m_driverController2, 7)
        .onTrue(new InstantCommand(
            () -> m_pneumatic.toggleDSolenoid(7, Value.kForward),
            m_pneumatic));
				new JoystickButton(m_driverController2, 8)
				.onTrue(new InstantCommand(
						() -> m_pneumatic.toggleDSolenoid(7, Value.kReverse),
						m_pneumatic));
				new JoystickButton(m_driverController2, 10)
				.onTrue(new InstantCommand(
						() -> m_pneumatic.toggleSolenoid(5),
						m_pneumatic));
				new JoystickButton(m_driverController2, 5)
        .onTrue(new RunCommand(
            () -> m_arm.setArmStatus(1),
          	m_arm));
				new JoystickButton(m_driverController2, 5)
        .onFalse(new RunCommand(
            () -> m_arm.setArmStatus(0),
          	m_arm));	
				new JoystickButton(m_driverController2, 3)
        .onTrue(new RunCommand(
            () -> m_arm.setArmStatus(-1),
          	m_arm));		
				new JoystickButton(m_driverController2, 3)
        .onFalse(new RunCommand(
            () -> m_arm.setArmStatus(0),
          	m_arm));	
				new JoystickButton(m_driverController2, 12)
				.onTrue(new RunCommand(
						() -> m_arm.setElevatorStatus(1),
						m_arm));
				new JoystickButton(m_driverController2, 12)
				.onFalse(new RunCommand(
						() -> m_arm.setElevatorStatus(0),
						m_arm));

  }

	protected void configureSmartDashboard(){
		SmartDashboard.putNumber("Compressor Current", m_pneumatic.getCurrent());
		SmartDashboard.putBoolean("Compressor Status", m_pneumatic.getStatus());
		SmartDashboard.putBoolean("Compressor Pressure Switch Status", m_pneumatic.getPressureSwitchValue());
	}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(0.6125, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(1.25, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }
}
