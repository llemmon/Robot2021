/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;

public class AutoTurnToAngle extends CommandBase {

  private final DriveTrain m_driveTrain;
  private double targetAngle;
  private double turnSpeed;
  private double turnPower;

  private double angleDifference;
  private int counter = 4;

  /**
   * Creates a new AutoTurnToAngle.
   */
  public AutoTurnToAngle(double targetAngleIn, double turnSpeedIn) {

    this.m_driveTrain = RobotContainer.m_driveTrain; // get driveTrain object from RobotContainer
    this.targetAngle = targetAngleIn;   // in degrees
    this.turnSpeed = turnSpeedIn;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_driveTrain.resetGyro();  // reset gyro to 0 heading
    System.out.println("**starting AutoTurnToAngle target angle: " + targetAngle + " head: "+String.format("%.3f", m_driveTrain.getHeadingAngle()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    angleDifference = getAngleError();   // go check current robot heading vs target angle
    if (angleDifference >= 0) {
      turnPower = turnSpeed;
    } else {
      turnPower = turnSpeed * -1;
    }
    m_driveTrain.doArcadeDrive(0, turnPower);  // turn using arcade drive
  }

  // get difference between targetAngle and current heading
  public double getAngleError() {
      double angleError = 0;
      angleError = targetAngle - m_driveTrain.getHeadingAngle();  // get difference
      angleError -= (360 * Math.floor(0.5 + ((angleError) / 360.0)));  // round down if needed
      return angleError;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("**ending AutoTurnToAngle command");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (counter++ % 5 == 0) { System.out.println("**Turn isFinished() - angle diff: "+String.format("%.3f", angleDifference)+" tolerance: "+DriveConstants.kToleranceDegrees); }
    return (Math.abs(angleDifference) < DriveConstants.kToleranceDegrees);   // see if time to quit
  }
}
