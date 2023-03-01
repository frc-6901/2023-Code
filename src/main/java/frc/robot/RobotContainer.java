// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.LimelightAim;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LimelightManager;
import frc.robot.subsystems.PneumaticGrabber;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

  private final Arm m_arm = new Arm();

  private final LimelightManager m_limelight = new LimelightManager();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_navigatorController =
      new CommandXboxController(ControllerConstants.kNavigatorPort);

  private final CommandXboxController m_operatorController =
      new CommandXboxController(ControllerConstants.kOperatorPort);
      
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_navigatorController.getLeftY() * 0.3, ControllerConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_navigatorController.getLeftX() * 0.3, ControllerConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_navigatorController.getRightX() * 0.3, ControllerConstants.kDriveDeadband),
                false, false),
            m_robotDrive));

    m_arm.setDefaultCommand(
      new RunCommand(
          () -> {
            m_arm.setVoltage(4 * m_operatorController.getRightY());
          },
          m_arm));

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

    m_operatorController.y().onTrue(new LimelightAim(m_limelight, m_robotDrive)
      .andThen(m_arm.setAngle(30)
        .andThen(m_arm.setSpoolVoltage(2))
          .andThen(new WaitCommand(.3))
            .andThen(m_arm.setSpoolVoltage(0))
              .andThen(m_pneumaticGrabber.openGrabber())
                .andThen(new WaitCommand(0.3))
                  .andThen(m_pneumaticGrabber.closeGrabber())
                    .andThen(m_arm.setSpoolVoltage(-2))
                      .andThen(new WaitCommand(.3))
                        .andThen(m_arm.setSpoolVoltage(0))
            ));
              
    m_operatorController.x().whileTrue(new LimelightAim(m_limelight, m_robotDrive));

    boolean inManualMode = false;

    if (m_operatorController.a().getAsBoolean())
      inManualMode = true;
    if (m_operatorController.x().getAsBoolean() || m_operatorController.y().getAsBoolean())
      inManualMode = false;
    
    if (inManualMode) {
      if (m_operatorController.getHID().getPOV() <= 10 && m_operatorController.getHID().getPOV() >= 350) {
        m_arm.setSpoolVoltage(2);

      }
      if (m_operatorController.getHID().getPOV() >= 170 && m_operatorController.getHID().getPOV() <= 190) {
        m_arm.setSpoolVoltage(-2);
      }
      
      m_operatorController.leftTrigger().onTrue(m_arm.setAngle(0));
      m_operatorController.rightTrigger().onTrue(m_arm.setVoltage(-2));
    }
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
