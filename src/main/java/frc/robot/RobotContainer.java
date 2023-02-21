// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
//import frc.robot.commands.LimelightAim;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ExampleSubsystem;
//import frc.robot.subsystems.LimelightManager;
import frc.robot.subsystems.PneumaticGrabber;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.math.MathUtil;
/*import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;*/

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final PneumaticGrabber m_pneumaticGrabber = new PneumaticGrabber();

  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // private final Drivetrain m_drivetrain = new Drivetrain();

  private final Arm m_arm = new Arm();

  //private final LimelightManager m_limelight = new LimelightManager();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_navigatorController =
      new CommandXboxController(ControllerConstants.kOperatorPort);

  private final CommandXboxController m_operatorController =
      new CommandXboxController(ControllerConstants.kOperatorPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_navigatorController.getLeftY(), ControllerConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_navigatorController.getLeftX(), ControllerConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_navigatorController.getRightX(), ControllerConstants.kDriveDeadband),
                true, true),
            m_robotDrive);

    /*m_drivetrain.setDefaultCommand(
      new RunCommand(
          () -> {
            m_drivetrain.drive(
                -DrivetrainConstants.kDriveForwardMultiplier * m_navigatorController.getLeftY(),
                DrivetrainConstants.kDriveTurnMultiplier * m_navigatorController.getLeftX());
          },
          m_drivetrain));*/

    /*m_arm.setDefaultCommand(
      new RunCommand(
          () -> {
            m_arm.setVoltage(6 * m_operatorController.getRightY());
          },
          m_arm));*/

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_operatorController.a().onTrue(m_arm.setPosition(ArmConstants.position[0]));
    m_operatorController.b().onTrue(m_arm.setPosition(ArmConstants.position[1]));
    m_operatorController.y().onTrue(m_arm.setPosition(ArmConstants.position[2]));
    /*m_operatorController.x().whileTrue(new LimelightAim(m_limelight, m_robotDrive));*/
    
    m_operatorController.leftBumper().onTrue(m_pneumaticGrabber.openGrabber());
    m_operatorController.rightBumper().onTrue(m_pneumaticGrabber.closeGrabber());
  }

  /** 
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * zz 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
