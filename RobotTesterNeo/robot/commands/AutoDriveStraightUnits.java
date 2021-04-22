/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.text.MessageFormat;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class AutoDriveStraightUnits extends CommandBase {

  private final DriveTrain m_driveTrain;
  private double speed;
  private double distance;
  private PIDController pid;

  //private static final double kDeadbandRangeNeg= -0.02;
  //private static final double kDeadbandRangePos= 0.02;

  //private static int counter1 = 1;
  private static int counter2 = 1;
  private static int counter3 = 1;
  private static int counter4 = 1;
  //private static int counter5 = 1;

  /**
   * Creates a new AutoDriveStraightUnits.
   */
  public AutoDriveStraightUnits(double speedIn, double distanceIn) {

    this.m_driveTrain = RobotContainer.m_driveTrain;    // get driveTrain object from RobotContainer
    this.speed = speedIn;
    this.distance = distanceIn;   // in inches

    this.pid = new PIDController(0.05, 0, 0);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.m_driveTrain);  // means no other command can use subsystem when this command is running.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    System.out.println(MessageFormat.format("**Started {0}  distance: {1}", this.getName(), distance));
    m_driveTrain.resetGyro();       // reset the gyro to 0
    m_driveTrain.resetEncoders();   // reset the encoders to 0 position
    pid.reset();
    pid.setTolerance(1.0, 10.0);    // set tolerance for at setpoint check
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double currentHeading = m_driveTrain.getHeadingAngle();
    double pidValue = MathUtil.clamp(pid.calculate(currentHeading, 0), -0.3, 0.3);  // check if veered off 0 heading (straight ahead)

    double reduceFactor;
    double leftSpeed;
    double rightSpeed;
    if (pidValue >= -0.03 && pidValue <= 0.03) {
      leftSpeed = speed;  // no change in direction if almost straight ahead (heading +- 0.5 of straight head)
      rightSpeed = speed;
   } else if ( pidValue >= 0 ) {  // if pid is pos need to turn to left (slow left side)
       //reduceFactor = Math.max(0.99, 1 - pidValue);   // get factor to reduce speed
       reduceFactor = 0.992;  
       leftSpeed = speed * reduceFactor;   // slow left by pid value or set amount
       rightSpeed = speed;
       //if (counter2++ % 2 == 0) { System.out.println("**turn Left Correction: pid: "+String.format("%.3f", pidValue)+" L/R: "+String.format("%.3f", leftSpeed)+" ~ "+String.format("%.3f", rightSpeed)); }
       if (counter2++ % 2 == 0) { System.out.println("**turn Left Correction: factor: "+String.format("%.3f", reduceFactor)+" pid: "+String.format("%.3f", pidValue)+" L/R: "+String.format("%.3f", leftSpeed)+" ~ "+String.format("%.3f", rightSpeed)); }

   } else {   // else pid is neg so need to turn to right (slow right side)	
       //reduceFactor = Math.max(0.99, 1 + pidValue);   // get factor to reduce speed   
       reduceFactor = 0.992;
       leftSpeed = speed;
       rightSpeed = speed * reduceFactor;   // slow right by pid value or set amount
       if (counter3++ % 2 == 0) { System.out.println("**turn Right Correction: factor: "+String.format("%.3f", reduceFactor)+" pid: "+String.format("%.3f", pidValue)+" L/R: "+String.format("%.3f", leftSpeed)+" ~ "+String.format("%.3f", rightSpeed)); }

   }
    m_driveTrain.doTankDrive(leftSpeed, rightSpeed);
  
    /***********************
    double leftSpeed = 0.0;
    double rightSpeed = 0.0;
    if ( pidValue >= -0.25 && pidValue <= 0.25) {
      leftSpeed = speed;    // close so no need for course correction
      rightSpeed = speed;
      if (counter1++ % 4 == 0) { System.out.println("**straight -pid: "+String.format("%.3f", pidValue)+" L/R: "+String.format("%.3f", leftSpeed)+" ~ "+String.format("%.3f", rightSpeed)); }
      //System.out.println("**straight ahead -pid: "+String.format("%.3f", pidValue)+" L/R: "+String.format("%.3f", leftSpeed)+" ~ "+String.format("%.3f", rightSpeed));

    } else if ( pidValue >= 0 ) {
      leftSpeed = speed * 0.95;    // need to turn to left (slow left side)
      rightSpeed = speed * 1.05;
      if (counter2++ % 2 == 0) { System.out.println("**turning to left Correction: pid: "+String.format("%.3f", pidValue)+" L/R: "+String.format("%.3f", leftSpeed)+" ~ "+String.format("%.3f", rightSpeed)); }
      //System.out.println("**turning to left Correction: pid: "+String.format("%.3f", pidValue)+" L/R: "+String.format("%.3f", leftSpeed)+" ~ "+String.format("%.3f", rightSpeed));

    } else {
      leftSpeed = speed * 1.05;  // need to turn to right (slow right side)
      rightSpeed = speed * 0.95;   
      if (counter3++ % 2 == 0) { System.out.println("**turning to Right Correction: pid: "+String.format("%.3f", pidValue)+" L/R: "+String.format("%.3f", leftSpeed)+" ~ "+String.format("%.3f", rightSpeed)); }
      //System.out.println("**turning to Right Correction: pid: "+String.format("%.3f", pidValue)+" L/R: "+String.format("%.3f", leftSpeed)+" ~ "+String.format("%.3f", rightSpeed));

    }
    //-------
    try { Thread.sleep(1000); } catch (Exception e) { System.out.println(e); } //sleep in milliseconds
   double test1 = pid.calculate( 7, 0 );
    System.out.println("**test1 pidOut for heading 7, targetAngle: 0 >  " + String.format("%.4f", test1));
    double test2 = pid.calculate( 6, 0 );
    System.out.println("**test2 pidOut for heading 6, targetAngle: 0 >  " + String.format("%.4f", test2));
    double test3 = pid.calculate( 5, 0 );
    System.out.println("**test3 pidOut for heading 5, targetAngle: 0 >  " + String.format("%.4f", test3));
    double test4 = pid.calculate( 4, 0 );
    System.out.println("**test4 pidOut for heading 4, targetAngle: 0 >  " + String.format("%.4f", test4));
    double test5 = pid.calculate( 3, 0 );
    System.out.println("**test5 pidOut for heading 3, targetAngle: 0 >  " + String.format("%.4f", test5));
    double test6 = pid.calculate( 2, 0 );
    System.out.println("**test6 pidOut for heading 2, targetAngle: 0 >  " + String.format("%.4f", test6));
    double test7 = pid.calculate( 1, 0 );
    System.out.println("**test7 pidOut for heading 1, targetAngle: 0 >  " + String.format("%.4f", test7));
    double test8 = pid.calculate( 0, 0 );
    System.out.println("**test8 pidOut for heading 0, targetAngle: 0 >  " + String.format("%.4f", test8));

    double test11 = pid.calculate( -1, 0 );
    System.out.println("**test11 pidOut for heading -1, targetAngle: 0 >  " + String.format("%.4f", test11));
    double test12 = pid.calculate( -2, 0 );
    System.out.println("**test12 pidOut for heading -2, targetAngle: 0 >  " + String.format("%.4f", test12));
    double test13 = pid.calculate( -3, 0 );
    System.out.println("**test13 pidOut for heading -3, targetAngle: 0 >  " + String.format("%.4f", test13));
    double test14 = pid.calculate( -4, 0 );
    System.out.println("**test14 pidOut for heading -4, targetAngle: 0 >  " + String.format("%.4f", test14));
    double test15 = pid.calculate( -5, 0 );
    System.out.println("**test15 pidOut for heading -5, targetAngle: 0 >  " + String.format("%.4f", test15));
    double test16 = pid.calculate( -6, 0 );
    System.out.println("**test16 pidOut for heading -6, targetAngle: 0 >  " + String.format("%.4f", test16));
    double test17 = pid.calculate( -7, 0 );
    System.out.println("**test17 pidOut for heading -7, targetAngle: 0 >  " + String.format("%.4f", test17));

    try { Thread.sleep(1000); } catch (Exception e) { System.out.println(e); } //sleep in milliseconds
    **/
  }

  private boolean isDistanceMet() {
    double currentDistance = m_driveTrain.getLeftDistanceInch();
    if (counter4++ % 3 == 0) { System.out.println("**isDistanceMet() check - current dist: " + String.format("%.3f", currentDistance)+"  target: "+distance); }
    if (Math.abs(currentDistance) >= distance) {
      System.out.println("**isDistanceMet(): true - current dist: " + String.format("%.3f", currentDistance)+"  target: "+distance);
      return true;
    } else {
      return false;
    }
  }

  public static double clampValue(double value, double min, double max) {
    return Math.max(min, Math.min(value, max));		// make sure within range
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //SmartDashboard.putString("Auto Command", "Finished AutoDriveStraightUnits");
    System.out.println(MessageFormat.format("**Ended {0}", this.getName()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isDone = isDistanceMet();
    return isDone;  // check if time to end
  }
}
