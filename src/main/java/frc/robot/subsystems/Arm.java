// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.armPosition;
import frc.utils.MagEncoderUtil;

public class Arm extends SubsystemBase {
  // Spark Max Motor Controller Object
  private CANSparkMax m_armMotorLeader =
      new CANSparkMax(ArmConstants.kArmMotorLeaderPort, MotorType.kBrushless);
  private CANSparkMax m_armMotorFollower =
      new CANSparkMax(ArmConstants.kArmMotorFollowerPort, MotorType.kBrushless);

  // Spark Max PID Controller Object
  private SparkMaxPIDController m_armController = m_armMotorLeader.getPIDController();

  // Spark Max Relative Encoder Object
  private RelativeEncoder m_encoder = m_armMotorLeader.getEncoder();

  // PID Coefficients
  public double kP = ArmConstants.kP;
  public double kD = ArmConstants.kD;
  public double kFF = ArmConstants.kFF;
  public double kMaxOutput = ArmConstants.kMaxOutput;
  public double kMinOutput = ArmConstants.kMinOutput;

  private double m_rotationPosition = 0.0;

  private WPI_TalonSRX m_spoolSRX = new WPI_TalonSRX(ArmConstants.kSpoolMotorPort);

  private double m_openLoopPower = 0.2;

  private double m_targetSpoolAngle = 0;

  private boolean positionMode = false;

  private armPosition m_armPosition = armPosition.START;

  private double m_spoolPosition;

  /** Creates a new Arm */
  public Arm() {
    m_armMotorLeader.restoreFactoryDefaults();
    m_armMotorFollower.restoreFactoryDefaults();
    m_armMotorFollower.setInverted(true);
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

    m_spoolSRX.configFactoryDefault();
    m_spoolSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    m_spoolSRX.setSensorPhase(false);
    m_spoolSRX.setInverted(false);
    m_spoolSRX.configForwardSoftLimitThreshold(MagEncoderUtil.distanceToNativeUnits(ArmConstants.kSpoolMotorForwardLimit, 1, 30));
    m_spoolSRX.configForwardSoftLimitEnable(true);
    m_spoolSRX.configReverseSoftLimitThreshold(MagEncoderUtil.distanceToNativeUnits(ArmConstants.kSpoolMotorReverseLimit, 1, 30));
    m_spoolSRX.configReverseSoftLimitEnable(true);
    m_spoolSRX.setSelectedSensorPosition(0.0);
    // m_spoolMotor.config_kP(0, ArmConstants.kSpoolP);
  }

  public CommandBase set(double percentPower) {
    return runOnce(
        () -> {
          m_openLoopPower = percentPower;
        });
  }

  public double getPositionDegrees() {
    return MagEncoderUtil.nativeUnitsToDistance(
        m_spoolSRX.getSelectedSensorPosition(), 1, 30);
  }

  public double getVelocityDegreesPerSec() {
    return MagEncoderUtil.nativeUnitsToVelocity(
        m_spoolSRX.getSelectedSensorVelocity(),1, 30);
  }

  public void setTargetPosition(double degrees) {
    m_targetSpoolAngle = degrees;
  }

  public boolean atTargetPosition() {
    return Math.abs(m_targetSpoolAngle - getPositionDegrees()) < 1;
  }

  @Override
  public void periodic() {
    if (positionMode) {
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double armAngle = SmartDashboard.getNumber("Arm Angle", 0);
    double rotations = SmartDashboard.getNumber("Set Rotations", 0);

    // Get desired armAngle. If not == to arm angle, rotate arm difference in degreees # of rotations
    // position = 0, 1, or 2
    // Each unit is 27 degrees
    // Each rotation is 9 degrees
    // A one unit difference in position will be 3 rotations
    // (Desired position - actual position) * 3 rotations = # of rotations
    double degreesPerRotation = 4.5;
    rotations = m_rotationPosition;
    SmartDashboard.putNumber("Arm Angle", m_rotationPosition);

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
    
    SmartDashboard.putNumber("Spool Position", getPositionDegrees());
    SmartDashboard.putNumber("Target Position", m_targetSpoolAngle);
    SmartDashboard.putNumber("Spool Position Rad", Units.degreesToRadians(getPositionDegrees()));

    SmartDashboard.putNumber("m_openLoopPower", m_openLoopPower);
      if (Math.abs(m_openLoopPower) > 0.25) {
        m_spoolSRX.set((m_openLoopPower > 0 ? 1 : -1) * 0.8);
      } else {
        m_spoolSRX.set(0);
      }

    m_spoolSRX.set(TalonSRXControlMode.Position, MagEncoderUtil.nativeUnitsToDistance(0.1, 1, 30));

    switch (m_armPosition) {
      case START:
        this.setAngle(0);
        m_spoolPosition = 0;
        break;
      case INTAKE:
        this.setAngle(10);
        m_spoolPosition = 0.1;
        break;
      case LOW:
        this.setAngle(15);
        m_spoolPosition = 0.15;
        break;
      case HIGH:
        this.setAngle(22);
        m_spoolPosition = 0.225;
        break;
    }
  }

  public CommandBase setAngle(double rotation) {
    return runOnce(
        () -> {
          m_rotationPosition = rotation;
        });
  }

  public CommandBase togglePositionMode() {
    return runOnce(
        () -> {
          positionMode = !positionMode;
        });
  }

  public CommandBase increaseArmAngle() {
    return runOnce(
        () -> {
          if (m_armPosition == armPosition.START)
            m_armPosition = armPosition.INTAKE;
          else if (m_armPosition == armPosition.INTAKE)
            m_armPosition = armPosition.LOW;
          else if (m_armPosition == armPosition.LOW)
            m_armPosition = armPosition.HIGH;
        });
  }

  public CommandBase decreaseArmAngle() {
    return runOnce(
        () -> {
          if (m_armPosition == armPosition.HIGH)
            m_armPosition = armPosition.LOW;
          else if (m_armPosition == armPosition.LOW)
            m_armPosition = armPosition.INTAKE;
          else if (m_armPosition == armPosition.INTAKE)
            m_armPosition = armPosition.START;
        });
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}