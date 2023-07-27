// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class XRPArm extends SubsystemBase {
  private Servo armServo;

  /** Creates a new XRPArm. */
  public XRPArm() {
    // Maps to XRP Servo 1
    armServo = new Servo(4);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setAngle(double angleDeg) {
    armServo.setAngle(angleDeg);
  }
}
