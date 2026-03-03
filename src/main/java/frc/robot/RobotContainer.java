// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OverbumpSubsystem;
import frc.robot.subsystems.ServoSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.auto.AutoBuilder;
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
  private final IntakeSubsystem m_intakeSubsystem = Robot.m_intakeSubsystem;
  private final OverbumpSubsystem m_overbumpSubsystem = new OverbumpSubsystem();
  private final ServoSubsystem m_servoSubsystem = Robot.m_servoSubsystem;
  private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(1);
  Joystick m_driverController2 = new Joystick(0);
  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    autoChooser = AutoBuilder.buildAutoChooser("Shooting Test");
    SmartDashboard.putData("Auto Chooser", autoChooser);

    
    

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY()* .5, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX()* .5, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX()* .5, OIConstants.kDriveDeadband),
                true),
            m_robotDrive));

   m_overbumpSubsystem.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_overbumpSubsystem.useOverbump(
                -MathUtil.applyDeadband(m_driverController.getRightY(), OIConstants.kDriveDeadband)),
            m_overbumpSubsystem));

    m_servoSubsystem.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_servoSubsystem.moveServo(),
            m_servoSubsystem));


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
    m_driverController.leftStick()
        .whileTrue(
        new RunCommand(() -> 
        m_robotDrive.setX(),
            m_robotDrive));

    m_driverController.start()
        .onTrue(
        new InstantCommand(() -> 
            m_robotDrive.zeroHeading(),
            m_robotDrive));

    m_driverController.rightTrigger()
        .whileTrue(  
        new RunCommand(() -> 
            m_intakeSubsystem.useIntake(.5),
            m_intakeSubsystem));
        
    
    m_driverController.rightTrigger()
        .whileFalse(
        new RunCommand(() -> 
            m_intakeSubsystem.useIntake(0),
            m_intakeSubsystem));
        
    
    m_driverController.rightBumper()
        .whileTrue(
        new RunCommand(() -> 
            m_intakeSubsystem.useIntake(-0.5),
            m_intakeSubsystem));
    
    m_driverController.rightBumper()
        .whileFalse(
        new RunCommand(() -> 
            m_intakeSubsystem.useIntake(0),
            m_intakeSubsystem));  
    
    new JoystickButton(m_driverController2, 9)
        .onTrue(
        new InstantCommand(() -> 
            m_intakeSubsystem.spinUp(-3000),
            m_intakeSubsystem));
    new JoystickButton(m_driverController2, 9)
        .onFalse(
        new InstantCommand(() -> 
            m_intakeSubsystem.spinUp(0),
            m_intakeSubsystem));

    m_driverController.leftBumper()
        .whileTrue(  
        new RunCommand(() -> 
            m_intakeSubsystem.useShooter(.5),
            m_intakeSubsystem));

     m_driverController.leftBumper()
        .whileFalse(  
        new RunCommand(() -> 
            m_intakeSubsystem.useShooter(0),
            m_intakeSubsystem));

    new JoystickButton(m_driverController2, 12)
        .onTrue(
        new InstantCommand(() -> 
            m_servoSubsystem.setServoTarget(-1),
            m_servoSubsystem));


    new JoystickButton(m_driverController2, 12)
        .onFalse(
        new InstantCommand(() -> 
            m_servoSubsystem.setServoTarget(1),
            m_servoSubsystem));

    new JoystickButton(m_driverController2, 4)
        .whileTrue(  
        new RunCommand(() -> 
            m_climbSubsystem.useClimber(.5),
            m_climbSubsystem));
    

    new JoystickButton(m_driverController2, 10)
        .whileTrue(  
        new RunCommand(() -> 
            m_climbSubsystem.useClimber(-.5),
            m_climbSubsystem));
    
    new JoystickButton(m_driverController2, 4)
        .whileFalse(  
        new RunCommand(() -> 
        {if (m_driverController2.getRawButton(10) == false){
            m_climbSubsystem.useClimber(0.0);
        }},
            m_climbSubsystem));
    
    new JoystickButton(m_driverController2, 10)
        .whileFalse(  
        new RunCommand(() -> 
        {if (m_driverController2.getRawButton(4) == false){
            m_climbSubsystem.useClimber(0.0);
        }},
            m_climbSubsystem));
    

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
   return autoChooser.getSelected();
}}
