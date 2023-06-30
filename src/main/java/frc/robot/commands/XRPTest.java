// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class XRPTest extends CommandBase {
  private final Drivetrain m_drivetrain;
  private double m_currSpeed = 0.0;
  private double m_delta = 0.1;
  private double m_lastUpdateTime = 0;

  /** Creates a new XRPTest. */
  public XRPTest(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_lastUpdateTime = Timer.getFPGATimestamp();
    m_currSpeed = 0;
    m_drivetrain.arcadeDrive(m_currSpeed, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Timer.getFPGATimestamp() - m_lastUpdateTime > 0.1) {
      m_currSpeed += m_delta;
      
      if (m_currSpeed > 1.0) {
        m_delta = -m_delta;
        m_currSpeed = 1.0;
      }
      else if (m_currSpeed < -1.0) {
        m_delta = -m_delta;
        m_currSpeed = -1.0;
      }
      m_lastUpdateTime = Timer.getFPGATimestamp();
    }
    m_drivetrain.arcadeDrive(m_currSpeed, 0);
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
