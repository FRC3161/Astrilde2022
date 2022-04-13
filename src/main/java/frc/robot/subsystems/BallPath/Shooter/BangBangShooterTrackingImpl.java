
package frc.robot.subsystems.BallPath.Shooter;

import java.time.Duration;
import java.util.concurrent.TimeUnit;

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

public class BangBangShooterTrackingImpl extends RepeatingIndependentSubsystem implements Shooter {
    
    private final CANSparkMax turretMotor;
    private final TalonFX shooterMotor;
    // private final TalonSRX hoodMotor;
    private final CANSparkMax hoodMotor;
    private final CANSparkMax hoodShooterMotor;

    private double leftLimit = -90.0;
    private double rightLimit = 90.0;

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
    private RelativeEncoder hoodShooterMotorEncoder;

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
    private final BangBangController hoodWheelControllerLoop;

    // ### TURRET ROTATION PID ###
    private SparkMaxPIDController turret_PIDController;
    public double turret_kP = 0.2;
    public double turret_kI = 0.00005;
    public double turret_kD = 0.08;
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
    private double setPointHoodShooterWheel = 0;
    int ballsOut = 0;
    boolean flipped = false;
    boolean ramped = false;
 
    public BangBangShooterTrackingImpl(CANSparkMax turretMotor, TalonFX shooterMotor, CANSparkMax hoodMotor, CANSparkMax hoodShooterMotor) {
        super(10, TimeUnit.MILLISECONDS);

        this.hoodShooterMotor = hoodShooterMotor;
        this.hoodShooterMotorEncoder = hoodShooterMotor.getEncoder();
        this.hoodWheelControllerLoop = new BangBang2Controller(1, 0.4, 500, Duration.ofMillis(500), 0.85);

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
        this.shooterControllerLoop = new BangBang2Controller(1, 0.45, 200, Duration.ofMillis(1_000), 0.85);
        SmartDashboard.putNumber("Shooter Set Speed", 0);
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
    }

    @Override
    public void defineResources(){
        require(turretMotor);
        require(shooterMotor);
        require(hoodMotor);
        require(hoodShooterMotor);
    }

    @Override
    public void setShotPosition(ShotPosition shotPosition) {
        this.requestedPosition = shotPosition;
    }

    public double getSetpointHood(double distance){
        double hoodDif, distDif, difFromUpper, percentToAdd, amountToAdd;
        // System.out.println("Distance" + distance);
        double returnAmount = 0;
        double[] distances = {55.0, 77, 100, 120, 145, 167, 202.95, 244.77, 305.66};
        int[] hoodValues = {  5,   65,  85,  100, 110, 130,    155,    165,    175};
        for (int i = 1; i < distances.length; i++) {
            double key = distances[i];
            if(distance < key){
                distDif = distances[i] - distances[i-1];
                hoodDif = hoodValues[i] - hoodValues[i-1];
                difFromUpper = distances[i] - distance;
                percentToAdd = difFromUpper / distDif;
                amountToAdd = percentToAdd * hoodDif;
                returnAmount = hoodValues[i]-amountToAdd;
                break;
            }
        }
        // SmartDashboard.getNumber("Hood Set Rotations", returnAmount);
        // System.out.println(returnAmount);

        return returnAmount;
    }

    
    public double getSetpointHoodShooter(Double distance){
        double wheelDif, distDif, difFromUpper, percentToAdd, amountToAdd;
        double returnAmount = 0;
        double[] distances = {44.0,    77,  113.4, 145.5, 170.8, 220.5};
        double[] wheelValues = {4000, 4000,  4000, 4000, 4000, 4000};
    
        for (int i = 1; i < distances.length; i++) {
            double key = distances[i];
            if(distance < key){
                distDif = distances[i] - distances[i-1];
                wheelDif = wheelValues[i] - wheelValues[i-1];
                difFromUpper = distances[i] - distance;
                percentToAdd = difFromUpper / distDif;
                amountToAdd = percentToAdd * wheelDif;
                returnAmount = wheelValues[i] - amountToAdd;
                break;
            }
        }
        return returnAmount;
    }

    public double getSetpointWheel(Double distance){
        double wheelDif, distDif, difFromUpper, percentToAdd, amountToAdd;
        double returnAmount = 0;
        double[] distances = {44.0,    77,  113.4, 145.5, 170.8, 220.5};
        int[] wheelValues = {5_500, 5_800,  6_225, 6_700, 8_200, 8_200};
    
        for (int i = 1; i < distances.length; i++) {
            double key = distances[i];
            if(distance < key){
                distDif = distances[i] - distances[i-1];
                wheelDif = wheelValues[i] - wheelValues[i-1];
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

    public static double canSeeTarget(){
        return canSeeTarget;
    }

    
    public CANSparkMax getHoodMotor(){
        return this.hoodMotor;
    }

    @Override
    public void task(){

        

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
        totalAngle = a1+y;
        totalAngleRadians = Math.toRadians(totalAngle);
        rs = Math.tan(totalAngleRadians);
        totalDistance = heightDif / rs;
        SmartDashboard.putNumber("Distance", totalDistance);
        

        switch (this.requestedPosition) {
            case FENDER:
                aim = false;
                setPointShooterFlywheel = 4400; //4700
                setPointHood = 0;
                setPointRotation = 0;
                shoot = true;
                setPointHoodShooterWheel = 4200; // 4250
                // System.out.println("FENDER");
                break;
            case GENERAL:
                aim = true;
                setPointHood = getSetpointHood(totalDistance);
                setPointShooterFlywheel = getSetpointWheel(totalDistance);
                shoot = true;
                setPointHoodShooterWheel = getSetpointHoodShooter(totalDistance);
                // System.out.println("GENERAL");
                break;
            case STOPAIM:
                aim = false;
                setPointShooterFlywheel = 0;
                setPointHood = 0;
                setPointRotation = 0;
                shoot = false;
                setPointHoodShooterWheel = 0;
                // System.out.println("STOPAIM");
                break;
            case STARTAIM:
                // System.out.println("STARTAIM");
            default:
                // System.out.println("DEFAULT");
                setPointHood = 0; // getSetpointHood(totalDistance);
                shoot = false;
                setPointShooterFlywheel = 0; // 4000; // TODO change to optimal value
                setPointHoodShooterWheel = 0; // 3000; // TODO change to optimal value
                aim = true;
                
                break;
        }

        double hoodShooterMotorVelocity = hoodShooterMotorEncoder.getVelocity();
        this.hoodShooterMotor.set(hoodWheelControllerLoop.calculate(hoodShooterMotorVelocity, setPointHoodShooterWheel));
        SmartDashboard.putNumber("Hood SHOOTER CURRENT VELOCITY", hoodShooterMotorVelocity);
        SmartDashboard.putNumber("Hood SHOOTER CURRRENT SETPOINT", setPointHoodShooterWheel);

        // hood position
        if (setPointHood > hoodEncoder.getPosition() - 1.5 && setPointHood < hoodEncoder.getPosition() + 1.5){
            hoodMotor.set(0);
        }
        else{
            // setting hood setpoint
            hood_PIDController.setReference(setPointHood, CANSparkMax.ControlType.kPosition);
            SmartDashboard.putNumber("Hood SetPoint", setPointHood);
        }

        // turret position
        if(aim){
            if(turretRotation > leftLimit && turretRotation < rightLimit){
                if(canSeeTarget == 1.0 && !flipRight && !flipLeft){
                    // System.out.println("can see");
                    seen = 0;
                    // System.out.println("Can see target and in limits");
                    if(x < rightLimitLimelight && x > leftLimitLimelight){
                        SmartDashboard.putNumber("Within limits", 1);
                        setPointRotation = turretRotation;
                    }else{
                        setPointRotation = turretRotation + x*conversion;
                        SmartDashboard.putNumber("Within limits", 0);
                        SmartDashboard.putNumber("x", x);
                        SmartDashboard.putNumber("x conversion", x*conversion);
                        SmartDashboard.putNumber("setPointRotation", setPointRotation);
                        SmartDashboard.putNumber("turretRotation", turretRotation);
                        if(setPointRotation <= leftLimit){
                            setPointRotation = rightLimit-1;
                            flipRight = true;
                        }else if(setPointRotation >= rightLimit){
                            setPointRotation = leftLimit+1;
                            flipLeft = true;
                        }
                    }
                }else if (seen > 20){

                    // System.out.println("Cannot see target");
                    // flip right
                    if(flipLeft){
                        if(canSeeTarget == 1.0){
                            if(turretRotation + x*conversion > leftLimit && turretRotation + x*conversion < rightLimit){
                                flipLeft = false;
                                setPointRotation = turretRotation + x*conversion;
                            }
                        
                        }else if (turretRotation <= (leftLimit+10)){
                            setPointRotation = rightLimit -1;
                            flipRight=true;
                            flipLeft = false;
                            
                        }
                    // flip left
                    }else if (flipRight){
                        if(canSeeTarget == 1.0){
                            if(turretRotation + x*conversion > leftLimit && turretRotation + x*conversion < rightLimit){
                                flipRight = false;
                                setPointRotation = turretRotation + x*conversion;
                            }
                        }else if (turretRotation >= rightLimit - 10){
                            setPointRotation = leftLimit+1;
                            flipLeft=true;
                            flipRight = false;
                            
                        }
                    }
                    else{
                        if(turretRotation > 0){
                            setPointRotation=leftLimit+1;
                            flipLeft=true;
                        }else{
                            setPointRotation=rightLimit-1;
                            flipRight = true;
                        }
                        
                    }
                }
                else{
                    seen ++;
                    // System.out.println("counting");
                }
                // flip left until you see a target?
            // just get back in the limits
            }else if(turretRotation < leftLimit){
                setPointRotation = leftLimit+1;
            }else if(turretRotation < rightLimit){
                setPointRotation = rightLimit-1;
            }
        }
        // System.out.println("Set point rotation" + setPointRotation);
        // System.out.println("Current turret Rotation" + turretRotation);
        turret_PIDController.setReference(setPointRotation, CANSparkMax.ControlType.kPosition);
        SmartDashboard.putNumber("Turret SetPoint", setPointRotation);


        // shooter wheel
        if(setPointShooterFlywheel != 0 && shoot){
            currentOutput = shooterControllerLoop.calculate(shooterEncoderReadingVelocity, setPointShooterFlywheel);
            // currentOutput += setPointShooterFlywheel == 0 ? 0 : 0.05; // hack "feed forward"
            currentOutput = Utils.normalizePwm(currentOutput);
        } else {
            currentOutput = 0;
        }
        this.shooterMotor.set(ControlMode.PercentOutput, currentOutput);

        // hood shooter wheel


    }    
    

    @Override
    public void findAndCenterTarget() {}

    @Override
    public void centerTarget(double tx){}

    @Override
    public void getDistance(double ty, double angle1, double angle2){}

    @Override
    public boolean readyToShoot(){
        boolean shooterReady = shooterControllerLoop.atSetpoint();
        boolean turretReady = false;
        boolean hoodReady = false;
        boolean hoodShooterReady = hoodWheelControllerLoop.atSetpoint();
        if(turretRotation > setPointRotation - 2.5 && turretRotation < setPointRotation + 2.5){
            turretReady = true;
        }
        if(hoodAngle > setPointHood - 1 && hoodAngle < setPointHood + 1){
            hoodReady = true;
        }
        SmartDashboard.putBoolean("SHOOTER READY", shooterReady);
        SmartDashboard.putBoolean("TURRET READY", turretReady);
        SmartDashboard.putBoolean("HOOD READY", hoodReady);
        SmartDashboard.putBoolean("HOOD SHOOTER READY", hoodShooterReady);

        return turretReady && shooterReady && hoodReady && hoodShooterReady;
    }

    public int getBallsShooter(){
        return ballsOut;
    }

    @Override
    public int checkBalls(){
        return 0;
    }

    @Override
    public void stopMotors(){
        setShotPosition(ShotPosition.NONE);
    }

    @Override
    public void resetSensors() {
        
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
