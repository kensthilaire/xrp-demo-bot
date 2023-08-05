// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.XRPArm;

public class ArmTest extends Command {
  private final XRPArm m_arm;
  
  private double m_currAngle = 0.0;
  private double m_angleDelta = 10.0;
  private double m_lastAngleChangeTime = 0;

  private final double ANGLE_CHANGE_STEP_TIME = 0.2;

  /** Creates a new ArmTest. */
  public ArmTest(XRPArm arm) {
    m_arm = arm;
    addRequirements(arm);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setAngle(180.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currTime = Timer.getFPGATimestamp();

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

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
