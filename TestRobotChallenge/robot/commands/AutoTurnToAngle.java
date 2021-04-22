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

  private double currentHeading;
  private double currentDiff;
  private boolean finished = false;
  private int counter = 1;

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
    finished = false;
    System.out.println("**starting AutoTurnToAngle target angle: " + targetAngle + " speed: " + turnSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double rotation;
    currentHeading = m_driveTrain.getHeadingAngle();
    currentDiff = targetAngle - Math.abs(currentHeading);  // check current robot heading vs target angle

    if (currentDiff >= -DriveConstants.kToleranceDegrees && currentDiff <= DriveConstants.kToleranceDegrees) {
      rotation = 0;
      finished = true;
      System.out.println("**done - diff: "+String.format("%.3f", currentDiff)+" heading: "+String.format("%.3f", currentHeading)+" target: "+targetAngle); 
    } else {
      if (targetAngle >= 0) {
        rotation = turnSpeed;
      } else {
        rotation = turnSpeed * -1;
      }
      if (counter++ % 2 == 0) { System.out.println("**currentDiff: "+String.format("%.3f", currentDiff)+" curr / target: "+String.format("%.3f", currentHeading)+" ~ "+String.format("%.3f", targetAngle)); }
    }
    m_driveTrain.doArcadeDrive(0, rotation);  // turn using arcadeDrive(xSpeed, zRotation)
  }

  // get difference between targetAngle and current heading
  //public double getAngleError() {
  //    double angleError = 0;
  //    angleError = targetAngle - m_driveTrain.getHeadingAngle();  // get difference
  //    angleError -= (360 * Math.floor(0.5 + ((angleError) / 360.0)));  // get shortest angle
  //   return angleError;
  //}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("**ending AutoTurnToAngle command");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return (currentDiff  >= -DriveConstants.kToleranceDegrees && currentDiff  <= DriveConstants.kToleranceDegrees);
    return finished;    // checked and set in execute()
  }
}
