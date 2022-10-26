
package frc.robot.subsystems.BallPath.Shooter;

import java.time.Duration;
import java.util.concurrent.TimeUnit;
import java.util.stream.Stream;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.robot.subsystem.RepeatingIndependentSubsystem;
import ca.team3161.lib.utils.Utils;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.BangBang2Controller;
import frc.robot.BangBangController;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.ScalarController;

public class BangBangShooterTrackingImpl extends RepeatingIndependentSubsystem implements Shooter {

    private final CANSparkMax turretMotor;
    private final TalonFX shooterMotor;
    // private final TalonSRX hoodMotor;
    private final CANSparkMax hoodMotor;
    // private final CANSparkMax hoodShooterMotor;

    private double leftLimit = Constants.Turret.leftLimit;
    private double rightLimit = Constants.Turret.rightLimit;

    private volatile ShotPosition requestedPosition = ShotPosition.NONE;
    private double setPointHood;
    private double setPointShooterFlywheel;
    private double setPointRotation;

    private double shooterEncoderReadingPosition;
    private double shooterEncoderReadingVelocity;
    private double hoodAngle;
    private double turretRotation;
    private RelativeEncoder hoodEncoder;
    private RelativeEncoder turretEncoder;
    // private RelativeEncoder hoodShooterMotorEncoder;

    private boolean shoot = false;
    private double currentOutput;
    private boolean flipRight = false;
    private boolean flipLeft = false;

    private final double leftLimitLimelight = -0.3;
    private final double rightLimitLimelight = 0.3;

    private final BangBangController shooterControllerLoop;
    boolean centerUsingLimelight = false;
    boolean aim = true;
    double currentPosition, error;
    double difference;
    int count;

    // turret hood motor pif
    private final ScalarController hoodWheelControllerLoop;

    // ### TURRET ROTATION PID ###
    private SparkMaxPIDController turret_PIDController;
    public double turret_kP = 0.21;
    public double turret_kI = 0.0000;
    public double turret_kD = 0.11;
    public double turret_kIz = 0;
    public double turret_kFF = 0;
    public double turret_kMaxOutput = 0.7;
    public double turret_kMinOutput = -0.7;
    // public double turret_kMaxOutputsearch = 0.3;
    // public double turret_kMinOutputsearch = 0.3;

    private SparkMaxPIDController hood_PIDController;
    public double hood_kP = 0.45;
    public double hood_kI = 0.000050;
    public double hood_kD = 0.05;
    public double hood_kIz = 0;
    public double hood_kFF = 0;
    public double hood_kMaxOutput = 0.3;
    public double hood_kMinOutput = -0.3;

    // LIMELIGHT STUFF
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry targetSkew = table.getEntry("ts");
    double x, y, a;
    static double canSeeTarget;
    double totalAngle;
    double rs;
    double totalDistance;
    double totalAngleRadians;

    double a1 = 35; // angle of limelight
    double a2 = y;
    // System.out.println(y);
    double h2 = 103; // HEIGHT OF TARGET
    double h1 = 35; // HEIGHT OF LIMELIGHT
    double conversion = 0.8333;

    double heightDif = h2 - h1;
    int seen = 0;
    long logItter = 0;
    // private double setPointHoodShooterWheel = 0;
    int ballsOut = 0;
    boolean flipped = false;
    boolean ramped = false;

    private int fenderSetPointHood = Constants.Turret.Fender.setPointHood;
    private int fenderSetPointRotation = Constants.Turret.Fender.setPointRotation;
    private int fenderSetPointShooterPID = Constants.Turret.Fender.setPointShooterPID;

    // Hood Distances and Values
    private double[] hoodDistances = Constants.Turret.HoodPoint.distances;
    private int[] hoodValues = Constants.Turret.HoodPoint.hoodValues;

    // Hood shooter distances and values
    private double[] shooterDistances = Constants.Turret.Shooter.distances;
    private int[] shooterValues = Constants.Turret.Shooter.wheelValues;

    public BangBangShooterTrackingImpl(CANSparkMax turretMotor, TalonFX shooterMotor, CANSparkMax hoodMotor) {
        super(10, TimeUnit.MILLISECONDS);

        // this.hoodShooterMotor =hoodShooterMotor;
        // this.hoodShooterMotorEncoder = hoodShooterMotor.getEncoder();
        this.hoodWheelControllerLoop = new ScalarController(10_000 / (4100 / 4000));
        // this.hoodWheelControllerLoop = new BangBangController(0.37, 400);
        // this.hoodWheelControllerLoop = new BangBang2Controller(1, 1, 300,
        // Duration.ofMillis(500), 0.75);

        // Turret Rotation
        this.turretMotor = turretMotor;
        this.turretEncoder = turretMotor.getEncoder();
        this.difference = turretEncoder.getCountsPerRevolution();
        this.turret_PIDController = turretMotor.getPIDController();
        turret_PIDController.setP(turret_kP);
        turret_PIDController.setI(turret_kI);
        turret_PIDController.setD(turret_kD);
        turret_PIDController.setIZone(turret_kIz);
        turret_PIDController.setFF(turret_kFF);
        turret_PIDController.setOutputRange(turret_kMinOutput, turret_kMaxOutput);

        // Shooter Wheel
        this.shooterMotor = shooterMotor;
        this.shooterControllerLoop = new BangBangController(Constants.Turret.Shooter.power,
                Constants.Turret.Shooter.tolerance);
        // this.shooterControllerLoop = new BangBang2Controller(1, 0.8, 200,
        // Duration.ofMillis(500), 0.75);
        SmartDashboard.putNumber("Shooter Set Speed", 0);
        SmartDashboard.putBoolean("Shooter Wheel Ready", shooterControllerLoop.atSetpoint());
        SmartDashboard.putBoolean("Hood Wheel Ready", hoodWheelControllerLoop.atSetpoint());
        // Hood
        this.hoodMotor = hoodMotor;
        this.hoodEncoder = hoodMotor.getEncoder();
        this.hood_PIDController = hoodMotor.getPIDController();
        hood_PIDController.setP(hood_kP);
        hood_PIDController.setI(hood_kI);
        hood_PIDController.setD(hood_kD);
        hood_PIDController.setIZone(hood_kIz);
        hood_PIDController.setFF(hood_kFF);
        hood_PIDController.setOutputRange(hood_kMinOutput, hood_kMaxOutput);
        if (Robot.DEBUG) {
            // SmartDashboard.putNumber("fenderHoodShooterMotorSpeed",
            // this.fenderHoodShooterMotorSpeed);
            SmartDashboard.putNumber("fenderSetPointHood", this.fenderSetPointHood);
            SmartDashboard.putNumber("fenderSetPointRotation", this.fenderSetPointRotation);
            SmartDashboard.putNumber("fenderSetPointShooterPID", this.fenderSetPointShooterPID);

            SmartDashboard.putString("hood distances", this.arrayToString(this.hoodDistances));
            SmartDashboard.putString("hood wheels", this.arrayToString(this.hoodValues));
            SmartDashboard.putString("shooter distances", this.arrayToString(this.shooterDistances));
            SmartDashboard.putString("shooter values", this.arrayToString(this.shooterValues));
        }
    }

    public int[] stringToIntArray(String rawData) {
        String[] arrayRawData = rawData.split(",");
        return Stream.of(arrayRawData).mapToInt(Integer::parseInt).toArray();
    }

    public double[] stringToDoubleArray(String rawData) {
        String[] arrayRawData = rawData.split(",");
        return Stream.of(arrayRawData).mapToDouble(Double::parseDouble).toArray();
    }

    public String arrayToString(double[] array) {
        String output = "";
        for (int i = 0; i < array.length; i++) {
            if (i + 1 == array.length) {
                output += array[i];
                continue;
            }
            output += array[i] + ",";
        }
        return output;
    }

    public String arrayToString(int[] array) {
        String output = "";
        for (int i = 0; i < array.length; i++) {
            if (i + 1 == array.length) {
                output += array[i];
                continue;
            }
            output += array[i] + ",";
        }
        return output;
    }

    @Override
    public void defineResources() {
        require(turretMotor);
        require(shooterMotor);
        require(hoodMotor);
        // require(hoodShooterMotor);
    }

    @Override
    public void setShotPosition(ShotPosition shotPosition) {
        this.requestedPosition = shotPosition;
    }

    public double getSetpointHood(double distance) {
        double hoodDif, distDif, difFromUpper, percentToAdd, amountToAdd;
        // System.out.println("Distance" + distance);
        double returnAmount = 0;
        double[] distances = this.hoodDistances;
        int[] hoodValues = this.hoodValues;

        for (int i = 1; i < distances.length; i++) {
            double key = distances[i];
            if (distance < key) {
                distDif = distances[i] - distances[i - 1];
                hoodDif = hoodValues[i] - hoodValues[i - 1];
                difFromUpper = distances[i] - distance;
                percentToAdd = difFromUpper / distDif;
                amountToAdd = percentToAdd * hoodDif;
                returnAmount = hoodValues[i] - amountToAdd;
                break;
            }
        }
        // SmartDashboard.getNumber("Hood Set Rotations", returnAmount);
        // System.out.println(returnAmount);

        return returnAmount;
    }

    public double getSetpointHoodShooter(Double distance) {
        double wheelDif, distDif, difFromUpper, percentToAdd, amountToAdd;
        double returnAmount = 0;
        double[] distances = Constants.Turret.HoodShooter.distances;
        int[] wheelValues = Constants.Turret.HoodShooter.wheelValues;

        for (int i = 1; i < distances.length; i++) {
            double key = distances[i];
            if (distance < key) {
                distDif = distances[i] - distances[i - 1];
                wheelDif = wheelValues[i] - wheelValues[i - 1];
                difFromUpper = distances[i] - distance;
                percentToAdd = difFromUpper / distDif;
                amountToAdd = percentToAdd * wheelDif;
                returnAmount = wheelValues[i] - amountToAdd;
                break;
            }
        }
        return returnAmount;
    }

    public double getSetpointWheel(Double distance) {
        double wheelDif, distDif, difFromUpper, percentToAdd, amountToAdd;
        double returnAmount = 0;
        double[] distances = this.shooterDistances;
        int[] wheelValues = this.shooterValues;

        for (int i = 1; i < distances.length; i++) {
            double key = distances[i];
            if (distance < key) {
                distDif = distances[i] - distances[i - 1];
                wheelDif = wheelValues[i] - wheelValues[i - 1];
                difFromUpper = distances[i] - distance;
                percentToAdd = difFromUpper / distDif;
                amountToAdd = percentToAdd * wheelDif;
                returnAmount = wheelValues[i] - amountToAdd;
                break;
            }
        }
        SmartDashboard.putNumber("RETURN AMOUNT", returnAmount);
        return returnAmount;
    }

    public static double canSeeTarget() {
        return canSeeTarget;
    }

    public CANSparkMax getHoodMotor() {
        return this.hoodMotor;
    }

    @Override
    public void task() {

        // getting and posting encoder reading positions for turret, hood and shooter
        shooterEncoderReadingPosition = shooterMotor.getSelectedSensorPosition();
        shooterEncoderReadingVelocity = shooterMotor.getSelectedSensorVelocity();
        hoodAngle = hoodEncoder.getPosition();
        turretRotation = turretEncoder.getPosition();

        SmartDashboard.putNumber("Shooter Position", shooterEncoderReadingPosition);
        SmartDashboard.putNumber("Shooter Velocity", shooterEncoderReadingVelocity);
        SmartDashboard.putNumber("Turret Position", turretEncoder.getPosition());
        SmartDashboard.putNumber("Hood Position", hoodEncoder.getPosition());

        // getting limelight values and total distance
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        tv = table.getEntry("tv");
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        canSeeTarget = tv.getDouble(0.0);
        totalAngle = a1 + y;
        totalAngleRadians = Math.toRadians(totalAngle);
        rs = Math.tan(totalAngleRadians);
        totalDistance = heightDif / rs;
        SmartDashboard.putNumber("Distance", totalDistance);

        if (Robot.DEBUG) {
            // double fenderhoodshootermotorspeed =
            // SmartDashboard.getNumber("fenderHoodShooterMotorSpeed",
            // this.fenderHoodShooterMotorSpeed);
            int fendersetpointhood = (int) SmartDashboard.getNumber("fenderSetPointHood", this.fenderSetPointHood);
            int fendersetpointrotation = (int) SmartDashboard.getNumber("fenderSetPointRotation",
                    this.fenderSetPointRotation);
            int fendersetpointshooterPID = (int) SmartDashboard.getNumber("fenderSetPointShooterPID",
                    this.fenderSetPointShooterPID);

            if (fendersetpointshooterPID != this.fenderSetPointShooterPID) {
                this.fenderSetPointShooterPID = fendersetpointshooterPID;
            }
            if (fendersetpointhood != this.fenderSetPointHood) {
                this.fenderSetPointHood = fendersetpointhood;
            }
            if (fendersetpointrotation != this.fenderSetPointRotation) {
                this.fenderSetPointRotation = fendersetpointrotation;
            }
            // if (fenderhoodshootermotorspeed != this.fenderHoodShooterMotorSpeed) {
            // this.fenderHoodShooterMotorSpeed = fenderhoodshootermotorspeed;
            // }
            double[] smartDashboardHoodDistance = this
                    .stringToDoubleArray(SmartDashboard.getString("hood distances", ""));
            int[] smartDashboardHoodValues = this.stringToIntArray(SmartDashboard.getString("hood wheels", ""));
            double[] smartDashboardShooterDistances = this
                    .stringToDoubleArray(SmartDashboard.getString("shooter distances", ""));
            int[] smartDashboardShooterValues = this
                    .stringToIntArray(SmartDashboard.getString("shooter values", ""));

            for (int i = 0; i < this.hoodDistances.length; i++) {
                if (this.hoodDistances[i] != smartDashboardHoodDistance[i]) {
                    this.hoodDistances[i] = smartDashboardHoodDistance[i];
                }
            }

            for (int i = 0; i < this.hoodValues.length; i++) {
                if (this.hoodValues[i] != smartDashboardHoodValues[i]) {
                    this.hoodValues[i] = smartDashboardHoodValues[i];
                }
            }

            for (int i = 0; i < this.shooterDistances.length; i++) {
                if (this.shooterDistances[i] != smartDashboardShooterDistances[i]) {
                    this.shooterDistances[i] = smartDashboardShooterDistances[i];
                }
            }

            for (int i = 0; i < this.shooterValues.length; i++) {
                if (this.shooterValues[i] != smartDashboardShooterValues[i]) {
                    this.shooterValues[i] = smartDashboardShooterValues[i];
                }
            }
        }

        switch (this.requestedPosition) {
            case RESET:
                // if (Robot.DEBUG) {
                // this.resetSensors();
                // aim = false;
                // setPointShooterFlywheel = 0;
                // setPointHood = 0;
                // setPointRotation = 0;
                // shoot = false;
                // }
                break;
            case FENDER:
                aim = false;
                setPointShooterFlywheel = this.fenderSetPointShooterPID; // 4700
                setPointHood = this.fenderSetPointHood;
                setPointRotation = this.fenderSetPointRotation;
                shoot = true;
                // setPointHoodShooterWheel = 3200; // 4250
                // System.out.println("FENDER");
                break;
            case GENERAL:
                aim = true;
                setPointHood = getSetpointHood(totalDistance);
                setPointShooterFlywheel = getSetpointWheel(totalDistance);
                shoot = true;
                // setPointHoodShooterWheel = getSetpointHoodShooter(totalDistance);
                // System.out.println("GENERAL");
                break;
            case AUTO:
                aim = true;
                // setPointHoodShooterWheel = 4000;
                setPointShooterFlywheel = 4000;
                setPointHood = getSetpointHood(totalDistance);
                shoot = false;
            case STOPAIM:
                aim = false;
                setPointShooterFlywheel = 0;
                setPointHood = 0;
                setPointRotation = 0;
                shoot = false;
                // setPointHoodShooterWheel = 0;
                // System.out.println("STOPAIM");
                break;
            case STARTAIM:
                // System.out.println("STARTAIM");
            default:
                // System.out.println("DEFAULT");
                setPointHood = 0; // getSetpointHood(totalDistance);
                shoot = false;
                setPointShooterFlywheel = 1500; // 4000; // TODO change to optimal value
                // setPointHoodShooterWheel = 0; // 3000; // TODO change to optimal value
                aim = Constants.Turret.Default.aim;
                setPointRotation = 0;

                break;
        }

        // double hoodShooterMotorVelocity = hoodShooterMotorEncoder.getVelocity();
        // this.hoodShooterMotor
        // .set(hoodWheelControllerLoop.calculate(hoodShooterMotorVelocity,
        // setPointHoodShooterWheel));
        // SmartDashboard.putNumber("Hood SHOOTER CURRENT VELOCITY",
        // hoodShooterMotorVelocity);
        // SmartDashboard.putNumber("Hood SHOOTER CURRRENT SETPOINT",
        // setPointHoodShooterWheel);

        // hood position
        if (setPointHood > hoodEncoder.getPosition() - 3 && setPointHood < hoodEncoder.getPosition() + 3) {
            hoodMotor.set(0);
        } else {
            // setting hood setpoint
            hood_PIDController.setReference(setPointHood, CANSparkMax.ControlType.kPosition);
            SmartDashboard.putNumber("Hood SetPoint", setPointHood);
        }

        // turret position
        if (aim) {
            if (turretRotation > leftLimit && turretRotation < rightLimit) {
                if (canSeeTarget == 1.0 && !flipRight && !flipLeft) {
                    // System.out.println("can see");
                    seen = 0;
                    // System.out.println("Can see target and in limits");
                    if (x < rightLimitLimelight && x > leftLimitLimelight) {
                        SmartDashboard.putNumber("Within limits", 1);
                        setPointRotation = turretRotation;
                    } else {
                        setPointRotation = turretRotation + x * conversion;
                        SmartDashboard.putNumber("Within limits", 0);
                        SmartDashboard.putNumber("x", x);
                        SmartDashboard.putNumber("x conversion", x * conversion);
                        SmartDashboard.putNumber("setPointRotation", setPointRotation);
                        SmartDashboard.putNumber("turretRotation", turretRotation);
                        if (setPointRotation <= leftLimit) {
                            setPointRotation = rightLimit - 1;
                            flipRight = true;
                        } else if (setPointRotation >= rightLimit) {
                            setPointRotation = leftLimit + 1;
                            flipLeft = true;
                        }
                    }
                } else if (seen > 20) {

                    // System.out.println("Cannot see target");
                    // flip right
                    if (flipLeft) {
                        if (canSeeTarget == 1.0) {
                            if (turretRotation + x * conversion > leftLimit
                                    && turretRotation + x * conversion < rightLimit) {
                                flipLeft = false;
                                setPointRotation = turretRotation + x * conversion;
                            }

                        } else if (turretRotation <= (leftLimit + 10)) {
                            setPointRotation = rightLimit - 1;
                            flipRight = true;
                            flipLeft = false;

                        }
                        // flip left
                    } else if (flipRight) {
                        if (canSeeTarget == 1.0) {
                            if (turretRotation + x * conversion > leftLimit
                                    && turretRotation + x * conversion < rightLimit) {
                                flipRight = false;
                                setPointRotation = turretRotation + x * conversion;
                            }
                        } else if (turretRotation >= rightLimit - 10) {
                            setPointRotation = leftLimit + 1;
                            flipLeft = true;
                            flipRight = false;

                        }
                    } else {
                        if (turretRotation > 0) {
                            setPointRotation = leftLimit + 1;
                            flipLeft = true;
                        } else {
                            setPointRotation = rightLimit - 1;
                            flipRight = true;
                        }

                    }
                } else {
                    seen++;
                    // System.out.println("counting");
                }
                // flip left until you see a target?
                // just get back in the limits
            } else if (turretRotation < leftLimit) {
                setPointRotation = leftLimit + 1;
            } else if (turretRotation < rightLimit) {
                setPointRotation = rightLimit - 1;
            }
        }
        // System.out.println("Set point rotation" + setPointRotation);
        // System.out.println("Current turret Rotation" + turretRotation);
        turret_PIDController.setReference(setPointRotation, CANSparkMax.ControlType.kPosition);
        SmartDashboard.putNumber("Turret SetPoint", setPointRotation);

        // shooter wheel
        if (setPointShooterFlywheel != 0) {
            currentOutput = shooterControllerLoop.calculate(shooterEncoderReadingVelocity, setPointShooterFlywheel);
            // currentOutput += setPointShooterFlywheel == 0 ? 0 : 0.05; // hack "feed
            // forward"
            currentOutput = Utils.normalizePwm(currentOutput);
        } else {
            currentOutput = 0;
        }
        this.shooterMotor.set(ControlMode.PercentOutput, currentOutput);

        // hood shooter wheel

    }

    @Override
    public void findAndCenterTarget() {
    }

    @Override
    public void centerTarget(double tx) {
    }

    @Override
    public void getDistance(double ty, double angle1, double angle2) {
    }

    @Override
    public boolean readyToShoot() {
        boolean shooterReady = shooterControllerLoop.atSetpoint();
        boolean turretReady = true;
        boolean hoodReady = true;
        // boolean hoodShooterReady = hoodWheelControllerLoop.atSetpoint();
        if (turretRotation > setPointRotation - 2.5 && turretRotation < setPointRotation + 2.5) {
            turretReady = true;
        }
        if (hoodAngle > setPointHood - 2 && hoodAngle < setPointHood + 2) {
            hoodReady = true;
        }
        SmartDashboard.putNumber("hood setpoint", setPointHood);
        SmartDashboard.putBoolean("SHOOTER READY", shooterReady);
        SmartDashboard.putBoolean("TURRET READY", turretReady);
        SmartDashboard.putBoolean("HOOD READY", hoodReady);
        // SmartDashboard.putBoolean("HOOD SHOOTER READY", hoodShooterReady);

        // return turretReady && shooterReady && hoodReady && hoodShooterReady;
        return turretReady && shooterReady && hoodReady;
    }

    public int getBallsShooter() {
        return ballsOut;
    }

    @Override
    public int checkBalls() {
        return 0;
    }

    @Override
    public void stopMotors() {
        setShotPosition(ShotPosition.NONE);
    }

    @Override
    public void resetSensors() {
        this.hoodEncoder.setPosition(0);
    }

    @Override
    public void lifecycleStatusChanged(LifecycleEvent previous, LifecycleEvent current) {
        switch (current) {
            case ON_INIT:
                // this.hoodMotor.setSelectedSensorPosition(0);
                this.hoodEncoder.setPosition(0);
                this.turretEncoder.setPosition(0);
            case ON_AUTO:
            case ON_TELEOP:
            case ON_TEST:
                this.start();
                break;
            case ON_DISABLED:
                this.cancel();
                break;
            case NONE:
            default:
                this.cancel();
                this.hoodEncoder.setPosition(0);
                break;
        }
    }
}
