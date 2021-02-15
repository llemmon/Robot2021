/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.text.MessageFormat;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class AutoDriveStraightUnits extends CommandBase {

  private final DriveTrain m_driveTrain;
  private double speed;
  private double distance;
  private PIDController pid;

  private static final double kDeadbandRangeNeg= -0.02;
  private static final double kDeadbandRangePos= 0.02;

  private static int counter1 = 4;
  private static int counter2 = 4;
  private static int counter3 = 4;

  /**
   * Creates a new AutoDriveStraightUnits.
   */
  public AutoDriveStraightUnits(double speedIn, double distanceIn) {

    this.m_driveTrain = RobotContainer.m_driveTrain;    // get driveTrain object from RobotContainer
    this.speed = speedIn;
    this.distance = distanceIn;   // in inches

    this.pid = new PIDController(speed, 0, 0);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.m_driveTrain);  // means no other command can use subsystem when this command is running.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    System.out.println(MessageFormat.format("**Started {0}  distance: {1}", this.getName(), distance));
    m_driveTrain.resetGyro();
    m_driveTrain.resetEncoders();
    pid.reset();
    pid.setTolerance(1.0, 10.0);    // set tolerance for at setpoint check
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double currentHeading = m_driveTrain.getHeadingAngle();
    double pidValue = pid.calculate(currentHeading, 0) / 100;  // check if veered off 0 heading (straight ahead)
    double pidValue1 = clampValue(pidValue, -0.20, 0.20);     // clamp pid value to valid range

    double leftSpeed = 0.0;
    double rightSpeed = 0.0;
    if ( pidValue < kDeadbandRangeNeg ) {
      leftSpeed = clampValue(speed + pidValue1, -0.75, 1.0);  // make sure not > 1
      rightSpeed = speed - pidValue1;   // need to turn to right (slow right side)
      if (counter1 % 5 == 0) { System.out.println("**turning to Right Correction: pid: "+String.format("%.3f", pidValue1)+" L/R: "+String.format("%.3f", leftSpeed)+"~"+String.format("%.3f", rightSpeed)); }

    } else if ( pidValue > kDeadbandRangePos ) {
      leftSpeed = speed - pidValue1;    // need to turn to left (slow left side)
      rightSpeed = clampValue(speed + pidValue1, -0.75, 1.0);   // make sure not > 1
      if (counter2 % 5 == 0) { System.out.println("**turning to left Correction: pid: "+String.format("%.3f", pidValue1)+" L/R: "+String.format("%.3f", leftSpeed)+"~"+String.format("%.3f", rightSpeed)); } 
    } else {
      leftSpeed = speed;
      rightSpeed = speed;
    }
    m_driveTrain.doTankDrive(leftSpeed, rightSpeed);
  }

  private boolean isDistanceMet() {
    return Math.abs(m_driveTrain.getAveDistanceInch()) >= distance;
  }

  public static double clampValue(double value, double min, double max) {
    return Math.max(min, Math.min(value, max));		// make sure within range
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("Auto Command", "Finished AutoDriveStraightUnits");
    System.out.println(MessageFormat.format("**Ended {0}", this.getName()));
    //m_driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isDone = isDistanceMet();
    if (counter3++ % 5 == 0) { System.out.println("**AutoDriveStraightUnits is at distance: " + isDone); }
    return isDone;  // check if time to end
  }
}
