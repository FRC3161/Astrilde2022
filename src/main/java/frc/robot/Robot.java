// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import ca.team3161.lib.robot.TitanBot;
import ca.team3161.lib.robot.motion.drivetrains.SpeedControllerGroup;
import ca.team3161.lib.utils.controls.LogitechDualAction;
import ca.team3161.lib.utils.controls.SquaredJoystickMode;
import ca.team3161.lib.utils.controls.Gamepad.PressType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain.Drive;
import frc.robot.subsystems.Drivetrain.DriveImpl;
import frc.robot.subsystems.BallPath.BallPath;
import frc.robot.subsystems.BallPath.BallPathImpl;
import frc.robot.subsystems.BallPath.Elevator.Elevator;
import frc.robot.subsystems.BallPath.Elevator.ElevatorImpl;
import frc.robot.subsystems.BallPath.Intake.Intake;
import frc.robot.subsystems.BallPath.Intake.IntakeImpl;
import frc.robot.subsystems.BallPath.Shooter.Shooter;
import frc.robot.subsystems.BallPath.Shooter.ShooterImpl;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.ClimberImpl;

// Intake Imports
import com.revrobotics.ColorSensorV3;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Ultrasonic;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TitanBot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private Drive drive;
  private LogitechDualAction driverPad;
  private BallPath ballSubsystem;
  private Climber climberSubsystem;

  @Override
  public int getAutonomousPeriodLengthSeconds() {
    return 15;
  }
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotSetup() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // DRIVETRAIN COMPONENTS
    PWMSparkMax leftMotorController1 = new PWMSparkMax(RobotMap.NEO_LEFT_DRIVE_PORTS[0]);
    PWMSparkMax leftMotorController2 = new PWMSparkMax(RobotMap.NEO_LEFT_DRIVE_PORTS[1]);
    SpeedControllerGroup leftSide = new SpeedControllerGroup(leftMotorController1, leftMotorController2);

    PWMSparkMax rightMotorController1 = new PWMSparkMax(RobotMap.NEO_RIGHT_DRIVE_PORTS[0]);
    PWMSparkMax rightMotorController2 = new PWMSparkMax(RobotMap.NEO_RIGHT_DRIVE_PORTS[1]);
    SpeedControllerGroup rightSide = new SpeedControllerGroup(rightMotorController1, rightMotorController2);

    Encoder leftEncoder = new Encoder(RobotMap.LEFT_ENCODER_PORTS[0], RobotMap.LEFT_ENCODER_PORTS[1], false, Encoder.EncodingType.k2X);
    Encoder rightEncoder = new Encoder(RobotMap.RIGHT_ENCODER_PORTS[0], RobotMap.RIGHT_ENCODER_PORTS[1], false, Encoder.EncodingType.k2X);
    this.drive = new DriveImpl(leftSide, rightSide, leftEncoder, rightEncoder);

    // INTAKE COMPONENTS
    WPI_TalonSRX intakeMotorController = new WPI_TalonSRX(RobotMap.INTAKE_TALON_PORT);
    ColorSensorV3 leftColorSensor = new ColorSensorV3(RobotMap.LEFT_COLOR_SENSOR_PORT);
    ColorSensorV3 rightColorSensor = new ColorSensorV3(RobotMap.RIGHT_COLOR_SENSOR_PORT);
    Ultrasonic intakeSensor = new Ultrasonic(RobotMap.INTAKE_ULTRASONIC_PORTS[0], RobotMap.INTAKE_ULTRASONIC_PORTS[1]);
    Intake intake = new IntakeImpl(intakeMotorController, leftColorSensor, rightColorSensor, intakeSensor);
    
    // ELEVATOR COMPONENTS
    WPI_TalonSRX elevatorMotorController = new WPI_TalonSRX(RobotMap.ELEVATOR_TALON_PORT);
    Elevator elevator = new ElevatorImpl(elevatorMotorController);

    // SHOOTER COMPONENTS
    WPI_TalonFX shooterMotorController = new WPI_TalonFX(RobotMap.SHOOTER_TALON_PORT);
    VictorSPX hoodMotorController = new VictorSPX(RobotMap.HOOD_MOTOR_PORT);
    Shooter shooter = new ShooterImpl(shooterMotorController, hoodMotorController);
    // shooter.setNeutralMode(NeutralMode.Coast);

    // ELEVATOR SENSOR
    Ultrasonic elevatorSensor = new Ultrasonic(RobotMap.ELEVATOR_ULTRASONIC_PORTS[0], RobotMap.ELEVATOR_ULTRASONIC_PORTS[1]);
    
    // Driverpad impl
    this.driverPad = new LogitechDualAction(RobotMap.DRIVER_PAD_PORT);
    this.ballSubsystem = new BallPathImpl(intake, elevator, shooter, elevatorSensor);
    this.climberSubsystem = new ClimberImpl();

    // register lifecycle components
    registerLifecycleComponent(driverPad);
    registerLifecycleComponent(drive);
    registerLifecycleComponent(ballSubsystem);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousSetup() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousRoutine() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopSetup() {
    // TODO Set up bindings
    this.driverPad.setMode(ControllerBindings.LEFT_STICK, ControllerBindings.Y_AXIS, new SquaredJoystickMode());
    this.driverPad.setMode(ControllerBindings.RIGHT_STICK, ControllerBindings.X_AXIS, new SquaredJoystickMode());

    this.driverPad.bind(ControllerBindings.INTAKE_START, PressType.PRESS, () -> this.ballSubsystem.startIntake());
    this.driverPad.bind(ControllerBindings.INTAKE_STOP, PressType.PRESS, () -> this.ballSubsystem.stopIntake());
    this.driverPad.bind(ControllerBindings.INTAKE_REVERSE, PressType.PRESS, () -> this.ballSubsystem.reverseIntake());

    this.driverPad.bind(ControllerBindings.SPIN_UP, PressType.PRESS, () -> this.ballSubsystem.readyToShoot());
    this.driverPad.bind(ControllerBindings.SHOOT, PressType.PRESS, () -> this.ballSubsystem.startShooter(0.0));
    this.driverPad.bind(ControllerBindings.SHOOT, PressType.RELEASE, () -> this.ballSubsystem.stopShooter());

    this.driverPad.bind(ControllerBindings.CLIMBER_EXTEND, PressType.PRESS, () -> this.climberSubsystem.extendOuterClimber());
    this.driverPad.bind(ControllerBindings.CLIMBER_RETRACT, PressType.PRESS, () -> this.climberSubsystem.retractOuterClimber());
    this.driverPad.bind(ControllerBindings.CLIMBER_ROTATE, PressType.PRESS, () -> this.climberSubsystem.angleOuter(0.0));
  }

  

  /** This function is called periodically during operator control. */
  @Override
  public void teleopRoutine() {
    this.drive.driveArcade(this.driverPad.getValue(ControllerBindings.LEFT_STICK, ControllerBindings.Y_AXIS), this.driverPad.getValue(ControllerBindings.RIGHT_STICK, ControllerBindings.X_AXIS));

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledSetup() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledRoutine() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testSetup() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testRoutine() {}

  
}
