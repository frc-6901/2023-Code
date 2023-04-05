// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.GrabberConstants;
import frc.robot.Constants.ArmConstants.armPosition;

public class Arm extends SubsystemBase {
  // Spark Max Motor Controller Object
  private CANSparkMax m_armMotorLeader =
      new CANSparkMax(ArmConstants.kArmMotorLeaderPort, MotorType.kBrushless);
  private CANSparkMax m_armMotorFollower =
      new CANSparkMax(ArmConstants.kArmMotorFollowerPort, MotorType.kBrushless);

  private CANSparkMax m_spoolMotor = new CANSparkMax(22, MotorType.kBrushless);

  // Spark Max PID Controller Object
  private SparkMaxPIDController m_armController = m_armMotorLeader.getPIDController();
  private SparkMaxPIDController m_spoolController = m_spoolMotor.getPIDController();

  // Spark Max Relative Encoder Object
  private RelativeEncoder m_encoder = m_armMotorLeader.getEncoder();

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

  private double m_rotationPosition = 0.0;

  private armPosition m_armPosition = armPosition.START;

  private double controllerInput;

  private double spoolSpeed = 0.0;

  /** Creates a new Arm */
  public Arm() {
    m_armMotorLeader.restoreFactoryDefaults();
    m_armMotorFollower.restoreFactoryDefaults();
    m_armMotorFollower.setInverted(true);
    m_armMotorLeader.setIdleMode(IdleMode.kBrake);
    m_armMotorFollower.setIdleMode(IdleMode.kBrake);
    m_armMotorFollower.follow(m_armMotorLeader, true);

    // PID coefficients
    kP = .05; 
    kI = 0;
    kD = 0.04; 
    kIz = 0; 
    kFF = 0.0; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 500;

    // Smart Motion Coefficients
    maxVel = 200; // rpm
    maxAcc = 0.04;

    // set PID coefficients
    m_armController.setP(kP);
    m_armController.setI(kI);
    m_armController.setD(kD);
    m_armController.setIZone(kIz);
    m_armController.setFF(kFF);
    m_armController.setOutputRange(kMinOutput, kMaxOutput);

    /**
     * Smart Motion coefficients are set on a SparkMaxPIDController object
     * 
     * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
     * the pid controller in Smart Motion mode
     * - setSmartMotionMinOutputVelocity() will put a lower bound in
     * RPM of the pid controller in Smart Motion mode
     * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
     * of the pid controller in Smart Motion mode
     * - setSmartMotionAllowedClosedLoopError() will set the max allowed
     * error for the pid controller in Smart Motion mode
     */
    int smartMotionSlot = 0;
    m_armController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_armController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    m_armController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    m_armController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);

    // display Smart Motion coefficients
    SmartDashboard.putNumber("Max Velocity", maxVel);
    SmartDashboard.putNumber("Min Velocity", minVel);
    SmartDashboard.putNumber("Max Acceleration", maxAcc);
    SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);
    SmartDashboard.putNumber("Set Position", 0);
    SmartDashboard.putBoolean("Mode", false);

    /* Spool shit */
    m_spoolMotor.restoreFactoryDefaults();
    m_spoolMotor.setIdleMode(IdleMode.kBrake);
    m_spoolMotor.setSoftLimit(SoftLimitDirection.kForward, 5);
    m_spoolMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
    m_spoolMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_spoolMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
  }

  @Override
  public void periodic() {
    double rotations = m_rotationPosition;

    if (m_armPosition == armPosition.START)
      rotations = -0.7 + controllerInput * 2;
    if (m_armPosition == armPosition.LOW)
      rotations = -6;
    if (m_armPosition == armPosition.MID)
      rotations = -13.5;
    if (m_armPosition == armPosition.HIGH)
      rotations = -17;
    if (m_armPosition == armPosition.EXTRA_HIGH)
      rotations = -21;
    
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    //double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double maxV = SmartDashboard.getNumber("Max Velocity", 0);
    double minV = SmartDashboard.getNumber("Min Velocity", 0);
    double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
    double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_armController.setP(p); kP = p; }
    if((i != kI)) { m_armController.setI(i); kI = i; }
    if((d != kD)) { m_armController.setD(d); kD = d; }
    if((iz != kIz)) { m_armController.setIZone(iz); kIz = iz; }
    //if((ff != kFF)) { m_armController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_armController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    if((maxV != maxVel)) { m_armController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
    if((minV != minVel)) { m_armController.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
    if((maxA != maxAcc)) { m_armController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    if((allE != allowedErr)) { m_armController.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }

    double setPoint, processVariable;
    boolean mode = SmartDashboard.getBoolean("Mode", false);
    if(mode) {
      setPoint = SmartDashboard.getNumber("Set Velocity", 0);
      m_armController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
      processVariable = m_encoder.getVelocity();
    } else {
      setPoint = rotations;
      /**
       * As with other PID modes, Smart Motion is set by calling the
       * setReference method on an existing pid object and setting
       * the control type to kSmartMotion
       */
      m_armController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
      processVariable = m_encoder.getPosition();

      SmartDashboard.putNumber("SetPoint", setPoint);
      SmartDashboard.putNumber("Process Variable", processVariable);
      SmartDashboard.putNumber("Output", m_armMotorLeader.getAppliedOutput());
    }

    /* More spool shit */
    m_armController.setReference(spoolSpeed, CANSparkMax.ControlType.kVelocity);
  }

  public CommandBase setAngle(double rotation) {
    return runOnce(
        () -> {
          m_rotationPosition = rotation;
        });
  }

  public CommandBase increaseArmAngle() {
    return runOnce(
        () -> {
          if (m_armPosition == armPosition.HIGH)
            m_armPosition = armPosition.EXTRA_HIGH;
          if (m_armPosition == armPosition.MID)
            m_armPosition = armPosition.HIGH;
          if (m_armPosition == armPosition.LOW)
            m_armPosition = armPosition.MID;
          if (m_armPosition == armPosition.START) {
            m_armPosition = armPosition.LOW;}
        });
  }

  public CommandBase decreaseArmAngle() {
    return runOnce(
        () -> {
          if (m_armPosition == armPosition.LOW) {
            GrabberConstants.isStart = 0;
            m_armPosition = armPosition.START;}
          if (m_armPosition == armPosition.MID)
            m_armPosition = armPosition.LOW;
          if (m_armPosition == armPosition.HIGH)
            m_armPosition = armPosition.MID;
          if (m_armPosition == armPosition.EXTRA_HIGH)
            m_armPosition = armPosition.HIGH;
        });
  }

  public void manualIntake(double input) {
      controllerInput = input;
  }

  public CommandBase setSpoolSpeed(double speed) {
    return runOnce(
      () ->
        {
          spoolSpeed = speed;
        }
    );
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}