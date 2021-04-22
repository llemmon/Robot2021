/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;

public class AutoSpinToAnglePID extends CommandBase {

  private final DriveTrain m_driveTrain;
  private double targetAngle;
  private double turnPower;

  private PIDController pid;
  private static final double kP = 0.03;
  private static final double kI = 0.00;
  private static final double kD = 0.00;

  //private static final double kDeadbandRangeNeg= -0.02;
  //private static final double kDeadbandRangePos= 0.02;

  private static double currentHeading;
  private int counter1 = 1;
  private int counter2 = 1;

  /**
   * Creates a new AutoSpinToAnglePID.
   */
  public AutoSpinToAnglePID(double targetAngleIn, double turnPowerIn) {

    this.m_driveTrain = RobotContainer.m_driveTrain; // get driveTrain object from RobotContainer
    this.targetAngle = targetAngleIn;   // in degrees from current heading
    this.turnPower = turnPowerIn;

    this.pid = new PIDController(kP, kI, kD);    // create PID object 
 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_driveTrain.resetGyro();  // reset gyro so 0 is current heading
    pid.setSetpoint(targetAngle);
    pid.setTolerance(DriveConstants.kToleranceDegrees, 10.0);    // set tolerance around setpoint, targetAngle in this case
    pid.enableContinuousInput(-180.0, 180.0); // Enable continuous input in range from -180 to 180

    System.out.println("**starting AutoSpinToAngle target angle: " +targetAngle +" heading: "+String.format("%.3f", m_driveTrain.getHeadingAngle()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentHeading = m_driveTrain.getHeadingAngle();
    // clamp pid output to between -turnPower and +turnPower
    double pidOutput = MathUtil.clamp(pid.calculate(currentHeading, targetAngle), -turnPower, turnPower);

    double speed = 0.0;
    if (pidOutput >= -0.3 && pidOutput <= 0.3) {
        speed = turnPower * 0.75;   // slow down if close to target
    } else {
        speed = turnPower;          // else do speed requested 
    }
    if (targetAngle >= 0) {

      if (counter1++ % 2 == 0) { System.out.println("**turn Left Correction pidOut: "+String.format("%.3f  ", pidOutput)+" heading: "+String.format("%.3f", currentHeading)); }
      m_driveTrain.doTankDrive(-speed, speed); // turn to left ( reverse left side)
    } else {

        if (counter2++ % 2 == 0) { System.out.println("**turn Right Correction pidOut: "+String.format("%.3f  ", pidOutput)+" heading: "+String.format("%.3f", currentHeading)); }
        m_driveTrain.doTankDrive(speed, -speed); // turn to right ( reverse right side)
    }

    /***********************
    try { Thread.sleep(1000); } catch (Exception e) { System.out.println(e); } //sleep in milliseconds
    double test1 = pid.calculate( 25, -25 );
    System.out.println("**test1 pidOut for heading +25, targetAngle: -25 >  " + String.format("%.4f", test1));
    double test2 = pid.calculate( 25, -90 );
    System.out.println("**test2 pidOut for heading +25, targetAngle: -90 >  " + String.format("%.4f", test2));
    double test3 = pid.calculate( 25, -135 );
    System.out.println("**test3 pidOut for heading +25, targetAngle: -135 > " + String.format("%.4f", test3));
    double test4 = pid.calculate( 15, 25 );
    System.out.println("**test4 pidOut for heading +15, targetAngle:  25 >  " + String.format("%.4f", test4));
    double test5 = pid.calculate( 20, 25 );
    System.out.println("**test5 pidOut for heading +20, targetAngle:  25 >  " + String.format("%.4f", test5));
    double test6 = pid.calculate( 25, 25 );
    System.out.println("**test6 pidOut for heading +25, targetAngle:  25 >  " + String.format("%.4f", test6));
    double test7 = pid.calculate( 25, 90 );
    System.out.println("**test7 pidOut for heading +25, targetAngle:  90 >  " + String.format("%.4f", test7));
    double test8 = pid.calculate( 25, 135 );
    System.out.println("**test8 pidOut for heading +25, targetAngle:  135 > " + String.format("%.4f", test8));

    double test11 = pid.calculate( -25, -25 );
    System.out.println("\n**test11 pidOut for heading -25, targetAngle: -25 >  " + String.format("%.4f", test11));
    double test12 = pid.calculate( -25, -90 );
    System.out.println("**test12 pidOut for heading -25, targetAngle: -90 >  " + String.format("%.4f", test12));
    double test13 = pid.calculate( -25, -135 );
    System.out.println("**test13 pidOut for heading -25, targetAngle: -135 > " + String.format("%.4f", test13));
    double test14 = pid.calculate( -25, 25 );
    System.out.println("**test14 pidOut for heading -25, targetAngle:  25 >  " + String.format("%.4f", test14));
    double test15 = pid.calculate( -25, 90 );
    System.out.println("**test15 pidOut for heading -25, targetAngle:  90 >  " + String.format("%.4f", test15));
    double test16 = pid.calculate( -25, 135 );
    System.out.println("**test16 pidOut for heading -25, targetAngle:  135 > " + String.format("%.4f", test16));

    try { Thread.sleep(1000); } catch (Exception e) { System.out.println(e); } //sleep in milliseconds
    **/
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    System.out.println("**ending AutoSpinToAnglePID command  current heading: " + String.format("%.3f", currentHeading));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentDiff = targetAngle - currentHeading;
    //System.out.println("**diff: "+String.format("%.3f", currentDiff+" heading: "+String.format("%.3f", currentHeading)+" target: "+targetAngle); 
    return (currentDiff >= -DriveConstants.kToleranceDegrees && currentDiff <= DriveConstants.kToleranceDegrees);
  }
}
