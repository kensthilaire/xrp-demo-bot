// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.XRPArm;

public class XRPTest extends CommandBase {
  private final Drivetrain m_drivetrain;
  private final XRPArm m_arm;
  
  private double m_currSpeed = 0.0;
  private double m_delta = 0.2;
  private double m_lastRampUpdateTime = 0;
  private double m_lastStateTime = 0;
  private boolean m_isRamp;

  private double m_currAngle = 0.0;
  private double m_angleDelta = 5.0;
  private double m_lastAngleChangeTime = 0;

  private final double ANGLE_CHANGE_STEP_TIME = 0.2;

  private final double RAMP_TIME = 3.0;
  private final double PAUSE_TIME = 2.0;

  /** Creates a new XRPTest. */
  public XRPTest(Drivetrain drivetrain, XRPArm arm) {
    m_drivetrain = drivetrain;
    m_arm = arm;
    addRequirements(drivetrain);
    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_lastRampUpdateTime = Timer.getFPGATimestamp();
    m_lastStateTime = Timer.getFPGATimestamp();
    m_isRamp = true;
    m_currSpeed = 0;
    m_drivetrain.arcadeDrive(m_currSpeed, 0);
    m_arm.setAngle(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currTime = Timer.getFPGATimestamp();

    // Drivetrain Test
    if (m_isRamp) {
      // Check if we should check out of ramp mode
      if (currTime - m_lastStateTime > RAMP_TIME) {
        m_lastStateTime = currTime;
        m_isRamp = false;
      }
      else {
        // Handle the ramp
        if (currTime - m_lastRampUpdateTime > 0.1) {
          m_currSpeed += m_delta;
      
          if (m_currSpeed > 1.0) {
            m_delta = -m_delta;
            m_currSpeed = 1.0;
          }
          else if (m_currSpeed < -1.0) {
            m_delta = -m_delta;
            m_currSpeed = -1.0;
          }
          m_lastRampUpdateTime = currTime;
        }
        m_drivetrain.arcadeDrive(m_currSpeed, 0);
      }
    }
    else {
      m_drivetrain.arcadeDrive(0, 0);
      if (currTime - m_lastStateTime > PAUSE_TIME) {
        m_lastStateTime = currTime;
        m_isRamp = true;
      }
    }

    // Servo Test
    if (currTime - m_lastAngleChangeTime > ANGLE_CHANGE_STEP_TIME) {
      m_currAngle += m_angleDelta;
      if (m_currAngle > 180.0) {
        m_currAngle = 180.0;
        m_angleDelta = -m_angleDelta;
      }
      if (m_currAngle < 0.0) {
        m_currAngle = 0.0;
        m_angleDelta = -m_angleDelta;
      }
      m_arm.setAngle(m_currAngle);
      m_lastAngleChangeTime = currTime;
    }

    // if (Timer.getFPGATimestamp() - m_lastRampUpdateTime > 0.1) {
    //   m_currSpeed += m_delta;
      
    //   if (m_currSpeed > 1.0) {
    //     m_delta = -m_delta;
    //     m_currSpeed = 1.0;
    //   }
    //   else if (m_currSpeed < -1.0) {
    //     m_delta = -m_delta;
    //     m_currSpeed = -1.0;
    //   }
    //   m_lastRampUpdateTime = Timer.getFPGATimestamp();
    // }
    // m_drivetrain.arcadeDrive(m_currSpeed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
