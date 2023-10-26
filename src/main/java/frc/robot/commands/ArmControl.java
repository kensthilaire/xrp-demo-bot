// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class ArmControl extends Command {
  private final Arm m_arm;
  private final Supplier<Boolean> m_extendSupplier;
  private final Supplier<Boolean> m_retractSupplier;

  private double m_angle;

  /**
   * Creates a new ArmControl command. This command will extend or retract the
   * XXRPRP arm based on controller input. This command does not terminate.
   *
   * @param arm - the XRP arm instance to control
   * @param extendSupplier - button supplier to extend the arm to the full open position
   * @param retractSupplier - button supplier to retract the arm to starting position
   */
  public ArmControl(
      Arm arm,
      Supplier<Boolean> extendSupplier,
      Supplier<Boolean> retractSupplier
  ) {
    m_arm = arm;
    m_extendSupplier = extendSupplier;
    m_retractSupplier = retractSupplier;
    addRequirements(arm);

    m_angle = 0.0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // initialize the arm angle to the starting (closed) position
    m_angle = 0.0;
    m_arm.setAngle(m_angle);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double new_angle = m_angle;

    if ( m_extendSupplier.get() )
      // if the extend button has been pressed, open the arm to the fully open position
      new_angle = 180.0;
    else if ( m_retractSupplier.get() )
      // if the retract button has been pressed, return the arm to the fully closed position
      new_angle = 0.0;

    // apply the new arm position angle if it has changed
    if ( new_angle != m_angle ) {
      m_arm.setAngle(m_angle);
      m_angle = new_angle;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
