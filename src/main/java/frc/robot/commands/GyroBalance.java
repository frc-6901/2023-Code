// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GyroBalance extends PIDCommand {
  /** Creates a new LimelightAim. */
  public GyroBalance(WPI_PigeonIMU gyro, DriveSubsystem driveSubsystem) {
    super(
        // The controller that the command will use
        new PIDController(.01, 0, .01),
        // This should return the measurement
        () -> gyro.getPitch(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          driveSubsystem.drive(output / 16, 0, 0, false, false);
        });
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);

    // Configure additional PID options by calling `getController` here.

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
