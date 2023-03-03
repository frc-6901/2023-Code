// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.LimelightAim;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LimelightManager;
import frc.robot.subsystems.PneumaticGrabber;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

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
                -MathUtil.applyDeadband(m_navigatorController.getLeftY() * 0.6, ControllerConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_navigatorController.getLeftX() * 0.6, ControllerConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_navigatorController.getRightX() * 0.3, ControllerConstants.kDriveDeadband),
                false, false),
            m_robotDrive));

    /*m_arm.setDefaultCommand(
      new RunCommand(
          () -> {
            m_arm.setVoltage(-MathUtil.applyDeadband(m_operatorController.getLeftY() * 4, ControllerConstants.kDriveDeadband));
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

    /*m_operatorController.y().onTrue(new LimelightAim(m_limelight, m_robotDrive)
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
            ));*/
            
    m_operatorController.x().onTrue(m_arm.set(0.5));
    m_operatorController.x().onFalse(m_arm.set(0));
    m_operatorController.b().onTrue(m_arm.set(-0.5));
    m_operatorController.b().onFalse(m_arm.set(0));
    m_operatorController.a().onTrue(m_arm.togglePositionMode());
    m_operatorController.leftBumper().onTrue(m_arm.increaseArmAngle());
    m_operatorController.rightBumper().onTrue(m_arm.decreaseArmAngle());

    m_navigatorController.leftBumper().onTrue(m_pneumaticGrabber.openGrabber());
    m_navigatorController.rightBumper().onTrue(m_pneumaticGrabber.closeGrabber());

      
  }

  /** 
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * zz 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    TrajectoryConfig m_trajectoryConfig = new TrajectoryConfig(
        AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveConstants.kDriveKinematics);

      Trajectory m_trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
                new Translation2d(3, 0),
                new Translation2d(3, -1)),
        new Pose2d(2, -1, Rotation2d.fromDegrees(0)),
        m_trajectoryConfig);

      PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
      PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
      ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);



      SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        m_trajectory, 
        m_robotDrive::getPose, 
        DriveConstants.kDriveKinematics, 
        xController, 
        yController, 
        thetaController, 
        m_robotDrive::setModuleStates, 
        m_robotDrive);

        // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(m_trajectory.getInitialPose());
      
    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }
}
