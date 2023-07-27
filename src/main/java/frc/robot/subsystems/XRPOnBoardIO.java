// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This class represents the onboard IO of the XRP Reference Robot. This includes the
 * USER pushbutton and LED
 */
public class XRPOnBoardIO extends SubsystemBase {
  private final DigitalInput m_button = new DigitalInput(0);
  private final DigitalOutput m_led = new DigitalOutput(1);

  /**
   * Constructor
   */
  public XRPOnBoardIO() {
    // No need to do anything else. Unlike the Romi, there are no other configurable
    // I/O ports
  }

  /**
   * Gets if the USER button is pressed
   * @return
   */
  public boolean getUserButtonPressed() {
    return m_button.get();
  }

  /**
   * Sets the onboard LED
   * @param value
   */
  public void setLed(boolean value) {
    m_led.set(value);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
