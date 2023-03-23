// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.GyroBalance;
import frc.robot.commands.LimelightAim;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LimelightManager;
import frc.robot.subsystems.PneumaticGrabber;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.math.MathUtil;

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

  ParallelCommandGroup limelightTest = new ParallelCommandGroup(new LimelightAim(m_limelight, m_robotDrive), new SequentialCommandGroup(new WaitCommand(1.5), m_limelight.limelightIsGood()));

  //SequentialCommandGroup for auto-scoring mid cones
  SequentialCommandGroup midConeScorer = new SequentialCommandGroup(
      new InstantCommand(
            () -> m_robotDrive.drive(-0.3, 0, 0, false, false),
            m_robotDrive), 
      new WaitCommand(0.1),
      new InstantCommand(
            () -> m_robotDrive.drive(0, 0, 0, false, false),
            m_robotDrive), 
      m_limelight.turnOnLED(),
      m_arm.increaseArmAngle(),
      new WaitCommand(0.5),
      m_arm.increaseArmAngle(),
      new WaitCommand(0.5),
      m_arm.increaseArmAngle(),
      new WaitCommand(0.5),
      limelightTest,
      m_robotDrive.stop(),
      m_limelight.turnOffLED()
  );

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_navigatorController =
      new CommandXboxController(ControllerConstants.kNavigatorPort);

  private final CommandXboxController m_operatorController =
      new CommandXboxController(ControllerConstants.kOperatorPort);
      
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
      CommandScheduler.getInstance().schedule(m_limelight.turnOffLED());
    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_navigatorController.getLeftY(), ControllerConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_navigatorController.getLeftX(), ControllerConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_navigatorController.getRightX() * 0.4, ControllerConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

    m_arm.setDefaultCommand(
        new RunCommand(
            () -> m_arm.manualIntake(m_operatorController.getLeftY()), m_arm));

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.co bmmand.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));
    
    m_operatorController.y().onTrue(m_arm.set(0.5));
    m_operatorController.y().onFalse(m_arm.set(0));
    m_operatorController.a().onTrue(m_arm.set(-0.5));
    m_operatorController.a().onFalse(m_arm.set(0));
    m_operatorController.leftBumper().onTrue(m_arm.increaseArmAngle());
    m_operatorController.rightBumper().onTrue(m_arm.decreaseArmAngle());
    m_operatorController.back().onTrue(m_arm.resetArmAngle());
    m_operatorController.x().onTrue(new GyroBalance(m_robotDrive.getGyro(), m_robotDrive));
    m_operatorController.leftTrigger().onTrue(m_pneumaticGrabber.openGrabber());
    m_operatorController.rightTrigger().onTrue(m_pneumaticGrabber.closeGrabber());
    
    m_navigatorController.y().onTrue(m_robotDrive.zeroHeading());
    m_navigatorController.rightTrigger().onTrue(m_robotDrive.setSpeedPercent(0.55));
    m_navigatorController.rightTrigger().onFalse(m_robotDrive.setSpeedPercent(0.4));
    m_navigatorController.leftTrigger().onTrue(m_limelight.turnOffLED());
  }

  /** 
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * zz 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    SequentialCommandGroup Dock = new SequentialCommandGroup(
      m_robotDrive.zeroHeading(),
      new InstantCommand(       
            () -> m_robotDrive.drive(0.3, 0, 0, false, false),
            m_robotDrive), 
      new WaitCommand(0.3),
      new InstantCommand(
            () -> m_robotDrive.drive(0.4, 0, 0, false, false),
            m_robotDrive),
      new WaitCommand(3),
      new InstantCommand(
            () -> m_robotDrive.drive(0, 0, 0, false, false),
            m_robotDrive),
      new WaitCommand(3),
      new InstantCommand(
            () -> m_robotDrive.drive(0.5, 0, 0, false, false),
            m_robotDrive),
      new WaitCommand(1.4),
      new InstantCommand(
            () -> m_robotDrive.drive(-0.3, 0, 0, false, false),
            m_robotDrive),
      new WaitCommand(0.45),
      new InstantCommand(
            () -> m_robotDrive.drive(0, 0.1, 0, false, false),
            m_robotDrive),
      new WaitCommand(0.2),
      new InstantCommand(
            () -> m_robotDrive.drive(0, 0, 0, false, false),
            m_robotDrive));

    SequentialCommandGroup Straight = new SequentialCommandGroup(
      new InstantCommand(
            () -> m_robotDrive.drive(-0.4, 0, 0, false, false),
            m_robotDrive), 
      new WaitCommand(0.3),
      new InstantCommand(
            () -> m_robotDrive.drive(0.4, 0, 0, false, false),
            m_robotDrive), 
      new WaitCommand(0.3),
      new InstantCommand(
            () -> m_robotDrive.drive(-0.4, 0, 0, false, false),
            m_robotDrive), 
      new WaitCommand(0.3),
      new InstantCommand(
            () -> m_robotDrive.drive(0.6, 0, 0, false, false),
            m_robotDrive), 
      new WaitCommand(3),
      new InstantCommand(
            () -> m_robotDrive.drive(0, 0, 0, false, false),
            m_robotDrive));

    return Dock;
  }
}
