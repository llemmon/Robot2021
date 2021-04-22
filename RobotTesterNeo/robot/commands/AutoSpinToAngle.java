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

  private double currentHeading;
  private double currentDiff;
  private boolean finished = false;
  private int counter = 1;
  private int counter2 = 1;

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
    finished = false;
    System.out.println("**starting AutoSpinToAngle target angle: "+targetAngle+" heading: "+String.format("%.3f", m_driveTrain.getHeadingAngle()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentHeading = m_driveTrain.getHeadingAngle();
    currentDiff = targetAngle - Math.abs(currentHeading);  // check current robot heading vs target angle

    if (currentDiff >= -DriveConstants.kToleranceDegrees && currentDiff <= DriveConstants.kToleranceDegrees) {
      m_driveTrain.doTankDrive(0.0, 0.0);
      finished = true;
      System.out.println("**done - diff: "+String.format("%.3f", currentDiff)+" heading: "+String.format("%.3f", currentHeading)+" target: "+targetAngle);  
    } else if (targetAngle > 0 ) {
        if (counter++ % 2 == 0) { System.out.println("**turn Right Correction: angle diff: "+String.format("%.3f", currentDiff)); }
        m_driveTrain.doTankDrive(turnPower, -turnPower); // need to turn to right (slow right side)
    } else {
        if (counter2++ % 2 == 0) { System.out.println("**turn Left Correction: angle diff: "+String.format("%.3f", currentDiff)); }
        m_driveTrain.doTankDrive(-turnPower, turnPower); // turn to left (slow left side)
    }
  }

  // get difference between targetAngle and current heading
  //public double getAngleError() {
  //  double angleError = 0;
  //  angleError = targetAngle - m_driveTrain.getHeadingAngle();  // get difference
  //  angleError -= (360 * Math.floor(0.5 + ((angleError) / 360.0)));  // get value between -180 and +180 degrees
  // return angleError;
  //}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    System.out.println("**ending AutoSpinToAngle command");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return (currentDiff  >= -DriveConstants.kToleranceDegrees && currentDiff  <= DriveConstants.kToleranceDegrees);
    return finished;    // checked and set in execute()
  }

}