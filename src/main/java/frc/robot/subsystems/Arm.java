// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  // Spark Max Motor Controller Object
  private CANSparkMax m_armMotorLeader =
      new CANSparkMax(ArmConstants.kArmMotorLeaderPort, MotorType.kBrushless);
  private CANSparkMax m_armMotorFollower =
      new CANSparkMax(ArmConstants.kArmMotorFollowerPort, MotorType.kBrushless);

  private WPI_TalonSRX m_spoolMotor = new WPI_TalonSRX(0);

  // Spark Max PID Controller Object
  private SparkMaxPIDController m_armController = m_armMotorLeader.getPIDController();

  private PIDController m_spoolMotorController = new PIDController(.05, 0, 0);

  // Spark Max Relative Encoder Object
  private RelativeEncoder m_encoder = m_armMotorLeader.getEncoder();

  // PID Coefficients
  public double kP = 0.1;
  public double kD = 0.5;
  public double kFF = 0;
  public double kMaxOutput = 1;
  public double kMinOutput = -1;

  private double m_desiredAngle = 0.0;

  public enum armState {
    START, INTAKE, LOW, MID, HIGH
  }

  private armState m_armState = armState.START;

  /** Creates a new Arm */
  public Arm() {
    m_armMotorLeader.restoreFactoryDefaults();
    m_armMotorFollower.restoreFactoryDefaults();
    m_armMotorLeader.setInverted(true);
    m_armMotorLeader.setIdleMode(IdleMode.kBrake);
    m_armMotorFollower.setIdleMode(IdleMode.kBrake);
    m_armMotorFollower.follow(m_armMotorLeader, true);

    // Set PID coefficients
    m_armController.setP(kP);
    m_armController.setD(kD);
    m_armController.setFF(kFF);
    m_armController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Arm Angle", 0);
    SmartDashboard.putNumber("Set Rotations", 0);
  }

  @Override
  public void periodic() {
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double armAngle = SmartDashboard.getNumber("Arm Angle", 0);
    double rotations = SmartDashboard.getNumber("Set Rotations", 0);

    // Get desired arm angle
    switch (m_armState) {
      case START:
        m_desiredAngle = 0;
        m_spoolMotor.set(TalonSRXControlMode.Position, 1);
        break;
      case INTAKE:
        m_desiredAngle = 25;
        m_spoolMotor.set(TalonSRXControlMode.Position, 1);
        break;
      case LOW:
        m_desiredAngle = 50;
        m_spoolMotor.set(TalonSRXControlMode.Position, 1);
        break;
      case MID:
        m_desiredAngle = 70;
        m_spoolMotor.set(TalonSRXControlMode.Position, 1);
        break;
      case HIGH:
        m_desiredAngle = 90;
        m_spoolMotor.set(TalonSRXControlMode.Position, 1);
        break;
    }
    
    // Desired angle - current angle = difference in degrees
    // degrees per rotation = 4.5
    // rotations = difference / degrees per rotation
    double degreesPerRotation = 360 / 80;
    rotations = m_desiredAngle - armAngle / degreesPerRotation;
    SmartDashboard.putNumber("Arm Angle", m_desiredAngle);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_armController.setP(p); kP = p; }
    if((d != kD)) { m_armController.setD(d); kD = d; }
    if((ff != kFF)) { m_armController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_armController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max;
    }

    m_armController.setReference(rotations, CANSparkMax.ControlType.kPosition);
    
    SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("ProcessVariable", m_encoder.getPosition());
  }

  public CommandBase setAngle(double angle) {
    return runOnce(
        () -> {
          m_desiredAngle = angle;
        });
  }

  public CommandBase setSpoolVoltage(double voltage) {
    return runOnce(
        () -> {
          m_spoolMotor.setVoltage(voltage);
        });
  }

  public CommandBase setVoltage(double voltage) {
    return runOnce(
        () -> {
          m_armMotorLeader.setVoltage(voltage);
        });
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
