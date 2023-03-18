// note: refactor the code

//main imports
package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

// motor controllers
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

//pneumatics
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;

//joysticks
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;

//encoders
/*import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.CANSparkMax.SoftLimitDirection;*/

//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
/*import java.sql.Time;
import java.sql.Timestamp;
import java.text.BreakIterator;
import java.util.concurrent.TimeUnit;*/
//import com.ctre.phoenix.time.StopWatch;
// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.*;
//import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
//import com.ctre.phoenix.motorcontrol.can.VictorSPX;
// import com.ctre.phoenix.motorcontrol.can.*;
//import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkMaxPIDController; 
// import com.ctre.phoenix.signals.*;
//import edu.wpi.first.wpilibj.motorcontrol.MotorController;
// import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Robot extends TimedRobot {
  //Creating varibales for the motor controllers
  PWMVictorSPX driveLeftA = new PWMVictorSPX(0);
  PWMVictorSPX driveLeftB = new PWMVictorSPX(1);
  // left motor controllers
  MotorControllerGroup leftMotors = new MotorControllerGroup(driveLeftA, driveLeftB);

  PWMVictorSPX driveRightA = new PWMVictorSPX(8);
  PWMVictorSPX driveRightB = new PWMVictorSPX(9);
  // right motor controllers
  MotorControllerGroup rightMotors = new MotorControllerGroup(driveRightA, driveRightB);

  //differential drive
  DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

  // variables for the arm controls
  CANSparkMax armYAxis = new CANSparkMax(11, MotorType.kBrushless);
  WPI_TalonSRX armXAxis = new WPI_TalonSRX(3);

  //variables for the pneumatics system
  Compressor compressor = new Compressor(1, PneumaticsModuleType.CTREPCM);
  DoubleSolenoid solenoid = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 0, 1);

  //PneumaticsControlModule controlModule = new PneumaticsControlModule(1);

  // joysticks
  Joystick driverController = new Joystick(2);
  XboxController armController = new XboxController(0);

  //initialize the encoder
  //private RelativeEncoder yAxisEncoder;

  //Constants for controlling the arm. needs adjustments for this robot
  final double armTimeUp = 0.5;

  //current limit for the arm
  static final int ArmCurrentLimitA = 20;

  //Arm power output for y axis
  static final double ArmYOutputPower = 0.5;

  // Arm power output for x axis
  static final double ArmXOutputPower = 0.5;

  //time to move the arm
  static final double ArmExtendTime = 2.0;

  //Varibles needed for the code
  boolean armUp = true; //Arm initialized to up because that's how it would start a match
  boolean burstMode = false;
  double lastBurstTime = 0; 

  double autoStart = 0;
  boolean goForAuto = true;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  
   @Override

  //function for setting the initial conditions of all the hardware
  public void robotInit() {

    //initial conditions for the drive motors
    leftMotors.setInverted(true);
    rightMotors.setInverted(false);

    /*driveLeftA.setInverted(true);
    driveLeftB.setInverted(true);
    driveRightA.setInverted(false);
    driveRightB.setInverted(false);*/
    
    //initla conditions for the arm
    armYAxis.setInverted(true);
    armYAxis.setIdleMode(IdleMode.kBrake);
    armYAxis.setSmartCurrentLimit(ArmCurrentLimitA);
    ((CANSparkMax) armYAxis).burnFlash();
    armXAxis.setInverted(false);

    //initial conditions for the intake
    compressor.enableDigital();

    //add a thing on the dashboard to turn off auto if needed
    SmartDashboard.putBoolean("Go For Auto", false);
    goForAuto = SmartDashboard.getBoolean("Go For Auto", false);

    //encoders
    /*yAxisEncoder = armYAxis.getEncoder(Type.kHallSensor, 4096);
    yAxisEncoder.setPosition(0);

    armYAxis.enableSoftLimit(SoftLimitDirection.kForward, true);
    armYAxis.enableSoftLimit(SoftLimitDirection.kReverse, false);

    armYAxis.setSoftLimit(SoftLimitDirection.kForward, 0);*/
    

  }

  /**
   * *set the arm output power. Positive is out, negative is in
   * 
   * @param percent
   */

    //function to set the arm output power in the vertical direction
  public void setArmYAxisMotor(double percent) {
    armYAxis.set(percent);
    SmartDashboard.putNumber("armYAxis power(%)", percent);
    SmartDashboard.putNumber("arm motor current (amps)", armYAxis.getOutputCurrent());
    SmartDashboard.putNumber("arm motor temperature(C)", armYAxis.getMotorTemperature());
  }

  //function to set the arm output power in the horizontal direction
  public void setArmXAxisMotor(double percent) {
    armXAxis.set(percent);
    SmartDashboard.putNumber("armXaxis power(%)", percent);
    /*SmartDashboard.putNumber("armXAxis motor current (amps)", armXAxis.getVoltage());
    SmartDashboard.putNumber("armXAxis motor temperature(C)", armXAxis.getMotorTemperature());*/
  }
  
 /**
  * set the arm output power.
  *
  * @param percent desired speed
  * @param amps current limit
  */
  
  //function for starting autonomous
  @Override
  public void autonomousInit() {
    //get a time for auton start to do events based on time later
    autoStart = Timer.getFPGATimestamp();
    //check dashboard icon to ensure good to do auto
    goForAuto = SmartDashboard.getBoolean("Go For Auto", false);
  }

  //function that is called periodically during autonomous
  @Override
  public void autonomousPeriodic() {
    //arm control code for autonomous
    if(armUp){
      if(Timer.getFPGATimestamp() - lastBurstTime < armTimeUp){
        //armYAxis.set(armTravel1);
      }
      else{
        //armYAxis.set(armHoldUp);
      }
    }
    /*else{
      if(Timer.getFPGATimestamp() - lastBurstTime < armTimeDown){
        arm.set(-armTravel);
        //arm.set(VictorSPXControlMode.PercentOutput, -armTravel);
      }
      else{
        arm.set(-armHoldUp);
        //arm.set(VictorSPXControlMode.PercentOutput, -armHoldUp);
      }
    }*/
    
    //get time since start of auto then run drive code for autonomous
    double autoTimeElapsed =  autoStart - Timer.getFPGATimestamp();
    if(goForAuto){

      if(autoTimeElapsed < 5){
        //stop spitting out the ball and drive backwards *slowly* for three seconds
        leftMotors.set(-0.75);
        rightMotors.set(-0.75);
      } else {
        //do nothing for the rest of auto
        leftMotors.set(0);
        rightMotors.set(0);
      }
    }

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    //compressor.enableDigital();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //Set up arcade steer
    double forward = -driverController.getRawAxis(1);
    double turn = driverController.getRawAxis(4);
    
    // set up arcade drive
    drive.arcadeDrive(forward, turn);

    /*double driveLeftPower = forward - turn;
    double driveRightPower = forward + turn;

    driveLeftA.set(driveLeftPower);
    driveLeftB.set(driveLeftPower);
    driveRightA.set(driveRightPower);
    driveRightB.set(driveRightPower);*/
    
    //Code for the arm
    double armPower;

    // motion for the arm in the vertical direction
    if (armController.getLeftY() > 0.5) {
      //raise the arm
      armPower = ArmYOutputPower;
      //*armController.getLeftY();
    }
    else if (armController.getLeftY() < -0.5) {
      //lower the arm
      armPower = -ArmYOutputPower;
      //*armController.getLeftY();
    }
    else {
      //do nothing and let it sit where it is
      armPower = 0.0;
      armYAxis.setIdleMode(IdleMode. kBrake);
    }
    setArmYAxisMotor(armPower);
    
    // motion for the arm in the horizontal direction
    if (armController.getLeftTriggerAxis() > 0.5) {
      //extend the arm
      // we could set it to srmpower = armXOuptuPower x get left trigger axis ( test it on the pivot firs)
      armPower = ArmXOutputPower;
      //*armController.getLeftTriggerAxis();
    }
    else if (armController.getRightTriggerAxis() > 0.5) {
      //retract the arm
      armPower = -ArmXOutputPower;
      //*armController.getRightTriggerAxis();
    }
    else {
      // do nothing and let it sit where it is
      armPower = 0.0;
      //armXAxis.stopMotor();
      armXAxis.setNeutralMode(NeutralMode.Brake);
    }
    setArmXAxisMotor(armPower);

    //Intake controls

    //solenoid controls
    if(armController.getLeftBumperPressed()){

      //fire the air one way
      solenoid.set(DoubleSolenoid.Value.kForward);
      
    } else if(armController.getRightBumperPressed()){

      //fire the air the other way
      solenoid.set(DoubleSolenoid.Value.kReverse);
    }

    //compressor controls
    /*if (armController.getAButton()) {

      //enable the compressdor
      compressor.enableAnalog(0, 50);

    } else if (armController.getBButton()) {

      //disable the compressor
      compressor.disable();
    }*/

   }

  //function for disabling everything at the end of the game
  @Override
  public void disabledInit() {
    //On disable turn off everything
    //done to solve issue with motors "remembering" previous setpoints after reenable
    driveLeftA.set(0);
    driveLeftB.set(0);
    driveRightA.set(0);
    driveRightB.set(0);
    armYAxis.set(0);
    armXAxis.set(0);
  }
}