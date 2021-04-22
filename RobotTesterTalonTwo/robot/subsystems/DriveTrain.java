/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveTrain extends SubsystemBase {

  //// ----- Motor Controllers ----- /////
  // There are 4 separate motor controllers with 1 pwm channel per controller
  private final WPI_TalonSRX motorDriveLeft1 = new WPI_TalonSRX(DriveConstants.leftDrive1Id);
  private final WPI_TalonSRX motorDriveLeft2 = new WPI_TalonSRX(DriveConstants.leftDrive2Id);
  private final WPI_TalonSRX motorDriveRight1 = new WPI_TalonSRX(DriveConstants.rightDrive1Id);
  private final WPI_TalonSRX motorDriveRight2 = new WPI_TalonSRX(DriveConstants.rightDrive2Id);
  
  // define Speed Controller Groups and Differential Drive for use in drive train
  private final SpeedControllerGroup driveGroupLeft = new SpeedControllerGroup(motorDriveLeft1, motorDriveLeft2);
  private final SpeedControllerGroup driveGroupRight = new SpeedControllerGroup(motorDriveRight1, motorDriveRight2);
  private final DifferentialDrive differentialDrive = new DifferentialDrive(driveGroupLeft, driveGroupRight);

  // define encoders for tracking distance
  // private final Encoder m_leftEncoder = new Encoder(2, 4, false, EncodingType.k4X);
  // private final Encoder m_rightEncoder = new Encoder(1, 3, false, EncodingType.k4X);
  // private final Encoder m_leftEncoder = new Encoder(DriveConstants.LEFT_ENCODER_A, DriveConstants.LEFT_ENCODER_B);
  // private final Encoder m_rightEncoder = new Encoder(DriveConstants.RIGHT_ENCODER_A, DriveConstants.RIGHT_ENCODER_B);

    // navX Gyro on RoboRio
    private AHRS m_Gyro;

  private static final boolean kSquareInputs = true;
  private static final boolean kSkipGyro = false;
  private static int counter = 4; // for limiting display

  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() {

    motorDriveLeft1.configFactoryDefault(); // Clear any non default configuration/settings
    motorDriveLeft2.configFactoryDefault();
    motorDriveRight1.configFactoryDefault();
    motorDriveRight2.configFactoryDefault();

    // DifferentialDrive inverts right side by default, so no need to setInvert() here
    //differentialDrive.setRightSideInverted(true);
    motorDriveLeft1.setInverted(false); // Invert 1 side of robot so will drive forward
    motorDriveLeft2.setInverted(false);
    motorDriveRight1.setInverted(true);
    motorDriveRight2.setInverted(true);

    motorDriveLeft1.setNeutralMode(NeutralMode.Coast); // set neutral mode
    motorDriveLeft2.setNeutralMode(NeutralMode.Coast);
    motorDriveRight1.setNeutralMode(NeutralMode.Coast);
    motorDriveRight2.setNeutralMode(NeutralMode.Coast);

    SupplyCurrentLimitConfiguration supplyLimit = new SupplyCurrentLimitConfiguration(true, 30, 35, 1.0);
    motorDriveLeft1.configSupplyCurrentLimit(supplyLimit);
    motorDriveLeft2.configSupplyCurrentLimit(supplyLimit); // set current limits
    motorDriveRight1.configSupplyCurrentLimit(supplyLimit);
    motorDriveRight2.configSupplyCurrentLimit(supplyLimit);

    differentialDrive.setDeadband(0.03);
    differentialDrive.setSafetyEnabled(false);    // ***to avoid error 'differentialDrive not fed often enough'

    if (kSkipGyro) {
      m_Gyro = null;

    } else {

      // navX-MXP Gyro instantiation
      try {
        // Instantiate Gyro - communicate w/navX-MXP via the MXP SPI Bus
        // Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB
        // See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details
        m_Gyro = new AHRS(SPI.Port.kMXP);
  
      } catch (RuntimeException ex) {
          DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
      }
      while (m_Gyro.isCalibrating()) {
        try { Thread.sleep(500); } catch (Exception e) { System.out.println(e); } //sleep in milliseconds
        System.out.println("**gyro isCalibrating . . .");
      }
      //SmartDashboard.putBoolean("gyro connected", m_Gyro.isConnected());
      System.out.println("**gyro connected " + m_Gyro.isConnected());
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Tank style driving for the DriveTrain.
   *
   * @param left  Speed in range [-1,1]
   * @param right Speed in range [-1,1]
   */
  public void doTankDrive(double leftDrivePercent, double rightDrivePercent) {

    if (counter++ % 5 == 0) { System.out.println("**driveTrain power L-R: "+String.format("%.3f  ", leftDrivePercent)+" ~ "+String.format("%.3f  ", rightDrivePercent)); }

    //leftDrivePercent = leftDrivePercent * 0.75;
    //rightDrivePercent = rightDrivePercent * 0.75;
    // SquareInputs adjust inputs at low speeds so better control - note 1.0 * 1.0 is still 1
    differentialDrive.tankDrive(leftDrivePercent, rightDrivePercent, kSquareInputs); // send output to drive train
  
    // use this instead if get differentialDrive not updated often enough
    //motorDriveLeft1.set(leftDrivePercent);
    //motorDriveLeft2.set(leftDrivePercent);
    //motorDriveRight1.set(rightDrivePercent);
    //motorDriveRight2.set(rightDrivePercent);
  }

    /**
   * Arcade style driving for the DriveTrain.
   *
   * @param speed    Speed in range [-1,1]
   * @param rotation Rotation in range [-1,1]
   */
  public void doArcadeDrive(double speed, double rotation) {

    // SquareInputs adjust inputs at low speeds so better control - note 1.0 * 1.0 is still 1
    differentialDrive.arcadeDrive(speed, rotation, kSquareInputs);
  }

    // http://pdocs.kauailabs.com/navx-mxp/guidance/terminology (for pitch, roll, yaw, IMU terminology)
    //public double getRelativeAngle() {
    //  return ((m_Gyro.getAngle() * Math.PI / 180.0));
    //}
  
    //public double getGyroAngle() {
    //  double angle = ((m_Gyro.getAngle() * Math.PI / 180.0) % (2 * Math.PI));
    //  if (m_Gyro.getAngle() < 0) {
    //    angle += (2 * Math.PI);
    //  }
    //  return angle;
    //} // Converts m_Gyro Angle (0-360) to Radians (0-2pi)
  
    public double getHeadingAngle() {
    //if (counter3++ % 5 == 0) {
      //System.out.println("gyropitch " + String.format("%.3f", m_Gyro.getPitch()));
      //System.out.println("gyroroll  " + String.format("%.3f", m_Gyro.getRoll()));
      //System.out.println("gyroyaw   " + String.format("%.3f", m_Gyro.getYaw()));
      //System.out.println("gyrorawZ  " + String.format("%.3f", m_Gyro.getRawGyroZ()));
      //System.out.println("gyrorawY  " + String.format("%.3f", m_Gyro.getRawGyroY()));
      //System.out.println("gyrorawX  " + String.format("%.3f", m_Gyro.getRawGyroX()));
      //System.out.println("gyroangle " + String.format("%.3f", m_Gyro.getAngle()));
      //System.out.println("gyrorelat angle " + String.format("%.3f", getRelativeAngle()));
    //}
    //return Math.IEEEremainder(m_Gyro.getAngle(), 360.0) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    //return Math.IEEEremainder(m_Gyro.getYaw(), 360.0) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    return Math.IEEEremainder(m_Gyro.getYaw(), 360.0);
    }
  
    public double getYaw() {
      return m_Gyro.getYaw(); // get rotation around Z axis for current heading
    }
  
    public void resetGyro() {
        // "Zero" yaw (whatever direction sensor is pointing now becomes new "Zero" degrees
      m_Gyro.reset();
      m_Gyro.zeroYaw();   // yaw is only thing that can be reset, pitch and roll can't (see docs)
    }

  public void stop() {
    System.out.println("**in drivetrain stop");
    doTankDrive(0.0, 0.0);
  }
}
