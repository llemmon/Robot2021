/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

//import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveTrain extends SubsystemBase {

  //// ----- Motor Controllers ----- /////
  // for Neo motors - 4 separate motor controllers with 1 pwm channel per controller
  private final CANSparkMax motorDriveLeft1 = new CANSparkMax(DriveConstants.leftDrive1Id, MotorType.kBrushless);
  private final CANSparkMax motorDriveLeft2 = new CANSparkMax(DriveConstants.leftDrive2Id, MotorType.kBrushless);
  private final CANSparkMax motorDriveRight1 = new CANSparkMax(DriveConstants.rightDrive1Id, MotorType.kBrushless);
  private final CANSparkMax motorDriveRight2 = new CANSparkMax(DriveConstants.rightDrive2Id, MotorType.kBrushless);

  // for Talon SRX motors
  //private final WPI_TalonSRX motorDriveLeft1 = new WPI_TalonSRX(DriveConstants.leftDrive1Id);
  //private final WPI_TalonSRX motorDriveLeft2 = new WPI_TalonSRX(DriveConstants.leftDrive2Id);
  //private final WPI_TalonSRX motorDriveRight1 = new WPI_TalonSRX(DriveConstants.rightDrive1Id);
  //private final WPI_TalonSRX motorDriveRight2 = new WPI_TalonSRX(DriveConstants.rightDrive2Id);
  
  // define Speed Controller Groups and Differential Drive for use in drive train
  private final SpeedControllerGroup driveGroupLeft = new SpeedControllerGroup(motorDriveLeft1, motorDriveLeft2);
  private final SpeedControllerGroup driveGroupRight = new SpeedControllerGroup(motorDriveRight1, motorDriveRight2);
  private final DifferentialDrive differentialDrive = new DifferentialDrive(driveGroupLeft, driveGroupRight);

  // define encoders for tracking distance
  private CANEncoder leftDriveEncoder; // Declare the left encoder
  private CANEncoder rightDriveEncoder;   // Declare the right encoder

  // private final Encoder leftDriveEncoder = new Encoder(2, 4, false, EncodingType.k4X);
  // private final Encoder rightDriveEncoder = new Encoder(1, 3, false, EncodingType.k4X);
  // private final Encoder leftDriveEncoder = new Encoder(DriveConstants.LEFT_ENCODER_A, DriveConstants.LEFT_ENCODER_B);
  // private final Encoder rightDriveEncoder = new Encoder(DriveConstants.RIGHT_ENCODER_A, DriveConstants.RIGHT_ENCODER_B);

  // define limit switches
  private CANDigitalInput m_forwardLimit;
  private CANDigitalInput m_reverseLimit;

  // navX Gyro on RoboRio
  private AHRS m_Gyro;

  private static final boolean kSquareInputs = true;
  private static final boolean kSkipGyro = false;
  private static final boolean kSkipEncoder = false;
  private static int counter = 49; // for limiting display

  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() {

    // for Neo motors
    motorDriveLeft1.restoreFactoryDefaults();
    motorDriveLeft2.restoreFactoryDefaults();
    motorDriveRight1.restoreFactoryDefaults();
    motorDriveRight2.restoreFactoryDefaults();

    motorDriveLeft1.setIdleMode(IdleMode.kCoast); // set idle mode
    motorDriveLeft2.setIdleMode(IdleMode.kCoast);
    motorDriveRight1.setIdleMode(IdleMode.kCoast);
    motorDriveRight2.setIdleMode(IdleMode.kCoast);

    motorDriveLeft1.setSmartCurrentLimit(DriveConstants.kAmpsMax);  // set current limit
    motorDriveLeft2.setSmartCurrentLimit(DriveConstants.kAmpsMax);
    motorDriveRight1.setSmartCurrentLimit(DriveConstants.kAmpsMax);
    motorDriveRight2.setSmartCurrentLimit(DriveConstants.kAmpsMax);
    // end for Neo motors 

    /*
    // for Talon SRX
    motorDriveLeft1.configFactoryDefault(); // Clear any non default configuration/settings
    motorDriveLeft2.configFactoryDefault();
    motorDriveRight1.configFactoryDefault();
    motorDriveRight2.configFactoryDefault();

    motorDriveLeft1.setNeutralMode(NeutralMode.Coast); // set neutral mode
    motorDriveLeft2.setNeutralMode(NeutralMode.Coast);
    motorDriveRight1.setNeutralMode(NeutralMode.Coast);
    motorDriveRight2.setNeutralMode(NeutralMode.Coast);

    SupplyCurrentLimitConfiguration supplyLimit = new SupplyCurrentLimitConfiguration(true, 30, 35, 1.0);
    motorDriveLeft1.configSupplyCurrentLimit(supplyLimit);
    motorDriveLeft2.configSupplyCurrentLimit(supplyLimit); // set current limits
    motorDriveRight1.configSupplyCurrentLimit(supplyLimit);
    motorDriveRight2.configSupplyCurrentLimit(supplyLimit);
    // end for Talon SRX
    */

    motorDriveLeft1.setInverted(true); // set direction
    motorDriveLeft2.setInverted(true);
    motorDriveRight1.setInverted(true);
    motorDriveRight2.setInverted(true);
    //DifferentialDrive inverts right side by default, so no need to setInvert() here
    //differentialDrive.setRightSideInverted(true);
    
    differentialDrive.setSafetyEnabled(false);    // ***to avoid error 'differentialDrive not fed often enough'
    
    if (kSkipEncoder) {
      leftDriveEncoder = null;
      rightDriveEncoder = null;

    } else {
      //https://github.com/REVrobotics/SPARK-MAX-Examples/tree/master/Java
      leftDriveEncoder = motorDriveLeft1.getEncoder(); // Define the left encoder
      rightDriveEncoder = motorDriveRight1.getEncoder();   // Declare the right encoder
      leftDriveEncoder.setPositionConversionFactor(DriveConstants.kEncoderTicksPerRevolution); // set encoder constants
      rightDriveEncoder.setPositionConversionFactor(DriveConstants.kEncoderTicksPerRevolution);
      // leftDriveEncoder.setReverseDirection(true);
      // rightDriveEncoder.setReverseDirection(false);
      //resetEncoders(); // set encoders to 0

      // set limit switches - can be 1 of 2 polarities: normally open or normally closed dipending on direction to limit 
      m_forwardLimit = motorDriveLeft1.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyClosed);
      m_reverseLimit = motorDriveRight1.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyClosed);
      m_forwardLimit.enableLimitSwitch(false);
      m_reverseLimit.enableLimitSwitch(false);
      //boolean limitPressed = m_forwardLimit.get();  // returns true if switch is pressed (or not connected), false when released
    }

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

    //SmartDashboard.putBoolean("gyro connected", m_Gyro.isConnected());
    System.out.println("**gyro connected: " + m_Gyro.isConnected());
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

    if (counter++ % 100 == 0) { System.out.println("**driveTrain power L-R: " + leftDrivePercent +" - "+ rightDrivePercent); }

    leftDrivePercent = leftDrivePercent * 0.75;
    rightDrivePercent = rightDrivePercent * 0.75;
    // SquareInputs adjust inputs at low speeds so better control - note 1.0 * 1.0 is still 1
    differentialDrive.tankDrive(leftDrivePercent, rightDrivePercent, kSquareInputs); // send output to drive train
  
    // use this instead if get differentialDrive not updated often enough
    //motorDriveLeft1.set(leftDrivePercent);
    //motorDriveLeft2.set(leftDrivePercent);
    //motorDriveRight1.set(rightDrivePercent);
    //motorDriveRight2.set(rightDrivePercent);
  }

  public void doTankDriveDefault(double leftDrivePercent, double rightDrivePercent) {

    //if (counter++ % 50 == 0) { System.out.println("**default driveTrain power L-R: " + leftDrivePercent +" - "+ rightDrivePercent); }
    // SquareInputs adjust inputs at low speeds so better control - note 1.0 * 1.0 is still 1
    differentialDrive.tankDrive(leftDrivePercent, rightDrivePercent, kSquareInputs); // send output to drive train
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
  public double getRelativeAngle() {
    if (kSkipGyro) { return 0.0; }
    return ((m_Gyro.getAngle() * Math.PI / 180.0));
  }

  public double getGyroAngle() {
    if (kSkipGyro) { return 0.0; }
    double angle = ((m_Gyro.getAngle() * Math.PI / 180.0) % (2 * Math.PI));
    if (m_Gyro.getAngle() < 0) {
      angle += (2 * Math.PI);
    }
    return angle;
  } // Converts m_Gyro Angle (0-360) to Radians (0-2pi)

  //public double getHeadingAngle() {
  //  return m_Gyro.getAngle(); // get current heading
  //}

  public double getHeadingAngle() {
    if (kSkipGyro) { return 0.0; }
    return Math.IEEEremainder(m_Gyro.getAngle(), 360.0) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public double getYaw() {
    if (kSkipGyro) { return 0.0; }
    return m_Gyro.getYaw(); // get rotation around Z axis for current heading
  }

  public void resetGyro() {
    if (!kSkipGyro) {
      // "Zero" yaw (whatever direction sensor is pointing now becomes new "Zero" degrees
      m_Gyro.reset();
      m_Gyro.zeroYaw();   // yaw is only thing that can be reset, pitch and roll can't (see docs)
    }
  }

  // public void resetEncoder() {
  public void resetEncoders() {
    if (!kSkipEncoder) {
      leftDriveEncoder.setPosition(0);
      rightDriveEncoder.setPosition(0);
    }
  }

  public double getLeftEncoderCount() {
    if (kSkipEncoder) { return 0.0; }
    return leftDriveEncoder.getPosition();  // return position of encoder in units of revolutions
  }

  public double getRightEncoderCount() {
    if (kSkipEncoder) { return 0.0; }
    return rightDriveEncoder.getPosition();  // return position of encoder in units of revolutions
  }

  public double getLeftDistanceInch() {
    if (kSkipEncoder) { return 0.0; }
    return Math.PI * DriveConstants.WHEEL_DIAMETER * (getLeftEncoderCount() / DriveConstants.PULSES_PER_REVOLUTION);
  }

  public double getRightDistanceInch() {
    if (kSkipEncoder) { return 0.0; }
    return Math.PI * DriveConstants.WHEEL_DIAMETER * (getRightEncoderCount() / DriveConstants.PULSES_PER_REVOLUTION);
  }

  public double getAveDistanceInch() {
    if (kSkipEncoder) { return 0.0; }
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  }

  public void stop() {
    System.out.println("in drivetrain stop");
    doTankDrive(0.0, 0.0);
  }

}
