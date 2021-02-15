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

public class AutoSpinToAngle extends CommandBase {

  private final DriveTrain m_driveTrain;
  private double targetAngle;
  private double turnPower;

  private double angleDifference;
  private int counter = 9;
  private int counter2 = 9;
  private int counter3 = 9;

  /**
   * Creates a new AutoSpinToAngle
   */
  public AutoSpinToAngle(double targetAngleIn, double turnPowerIn) {
    
    this.m_driveTrain = RobotContainer.m_driveTrain; // get driveTrain object from RobotContainer
    this.targetAngle = targetAngleIn;   // in degrees from current heading
    this.turnPower = turnPowerIn;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_driveTrain.resetGyro();  // reset gyro so 0 is current heading
    System.out.println("**starting AutoSpinToAngle command target angle: " + targetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    angleDifference = getAngleError();   // go check current robot heading vs target angle
    if (angleDifference < 0 ) {
        if (counter++ % 10 == 0) { System.out.println("**turn Right Correction: angle diff: "+String.format("%.3f", angleDifference)); }
        m_driveTrain.doTankDrive(turnPower, -turnPower); // need to turn to right (slow right side)
    } else {
        if (counter2++ % 10 == 0) { System.out.println("**turn Left Correction: angle diff: "+String.format("%.3f", angleDifference)); }
        m_driveTrain.doTankDrive(-turnPower, turnPower); // turn to left (slow left side)
    }
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

    System.out.println("**ending AutoSpinToAngle command");
    //m_driveTrain.stop();    // make sure stopped before exiting
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (counter3++ % 10 == 0) { System.out.println("**AutoSpinToAngle isFinished() check - angle diff: "+String.format("%.3f", angleDifference)+" tolerance: "+DriveConstants.kToleranceDegrees); }
    return (Math.abs(angleDifference) < DriveConstants.kToleranceDegrees);   // see if time to quit
  }

}