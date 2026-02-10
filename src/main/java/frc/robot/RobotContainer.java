// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OverbumpSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import com.pathplanner.lib.commands.PathPlannerAuto;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_armSubsystem = new IntakeSubsystem();
  private final OverbumpSubsystem m_overbumpSubsystem = new OverbumpSubsystem();

  // The driver's controller
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getRawAxis(1)* .5, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRawAxis(0)* .5, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRawAxis(2)* .5, OIConstants.kDriveDeadband),
                true),
            m_robotDrive));

     m_overbumpSubsystem.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_overbumpSubsystem.useOverbump(
                -MathUtil.applyDeadband(((m_driverController.getRawAxis(3)+1)/2), OIConstants.kDriveDeadband)),
            m_overbumpSubsystem));
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
    new JoystickButton(m_driverController, 4)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(m_driverController, 8)
        .onTrue(new InstantCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));

    new JoystickButton(m_driverController, 3)
        .whileTrue(  new RunCommand(
            () -> m_armSubsystem.useIntake(.5),
            m_armSubsystem));
    
    new JoystickButton(m_driverController, 3)
        .whileFalse(  new RunCommand(
            () -> m_armSubsystem.useIntake(0),
            m_armSubsystem));

    new JoystickButton(m_driverController, 2)
        .whileTrue(  new RunCommand(
            () -> m_armSubsystem.spinUp(.5),
            m_armSubsystem));   
            
    new JoystickButton(m_driverController, 2)
        .whileFalse(  new RunCommand(
            () -> m_armSubsystem.spinUp(0),
            m_armSubsystem));

    new JoystickButton(m_driverController, 1)
        .whileTrue(  new RunCommand(
            () -> m_armSubsystem.useShooter(.5),
            m_armSubsystem));

     new JoystickButton(m_driverController, 1)
        .whileFalse(  new RunCommand(
            () -> m_armSubsystem.useShooter(0),
            m_armSubsystem));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
   return new PathPlannerAuto("6809");
  }
}
