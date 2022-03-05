package frc.robot.subsystems.BallPath.Shooter;

import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.robot.subsystem.RepeatingIndependentSubsystem;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import ca.team3161.lib.utils.Utils;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PIDShooterImpl extends RepeatingIndependentSubsystem implements Shooter {

    private final TalonSRX turretMotor;
    private final TalonFX shooterMotor;
    private final TalonSRX hoodMotor;

    private double kp = 0.00075;
    private double ki = 0.0000475;
    private double kd = 0.000044;

    private volatile ShotPosition requestedPosition = ShotPosition.NONE;
    private int stallCount = 0;
    private final int stallCountLimit = 50; // 50 * 20ms = 1 second of stall
    private final double hoodStallCurrent = -13;
    private double setPointHood;
    private double setPointRotation;
    private double setPointShooterPID;

    private double shooterEncoderReadingPosition;
    private double shooterEncoderReadingVelocity;
    private double turretHoodPosition;
    private double turretEncoderReadingPosition;
    private double turretHoodVelocity;
    private double turretEncoderReadingVelocity;

    private boolean turretReady = false;
    private boolean hoodReady = false;

    private double currentOutput;

    private final double hoodBuffer = 10000;
    private final double turretBuffer = 30000;
    private final double turretSpeed = 0.3;
    private final double hoodSpeed = 0.5;

    private final PIDController shooterPid;

    public PIDShooterImpl(TalonSRX turretMotor, TalonFX shooterMotor, TalonSRX hoodMotor) {
        super(20, TimeUnit.MILLISECONDS);
        this.turretMotor = turretMotor;
        this.shooterMotor = shooterMotor;
        this.hoodMotor = hoodMotor;
        this.shooterPid = new PIDController(kp, ki, kd);
        this.shooterPid.setTolerance(200, 100);
    }

    @Override
    public void defineResources(){
        require(turretMotor);
        require(shooterMotor);
        require(hoodMotor);
    }

    @Override
    public void setShotPosition(ShotPosition shotPosition) {
        this.requestedPosition = shotPosition;
    }

    @Override
    public void task(){
        shooterEncoderReadingPosition = shooterMotor.getSelectedSensorPosition();
        shooterEncoderReadingVelocity = shooterMotor.getSelectedSensorVelocity();
        turretHoodPosition = hoodMotor.getSelectedSensorPosition();
        turretHoodVelocity = hoodMotor.getSelectedSensorVelocity();
        turretEncoderReadingPosition = this.turretMotor.getSelectedSensorPosition();
        turretEncoderReadingVelocity = this.turretMotor.getSelectedSensorVelocity();

        SmartDashboard.putNumber("Shooter Encoder reading position", shooterEncoderReadingPosition);
        SmartDashboard.putNumber("Shooter Encoder Reading Velocity", shooterEncoderReadingVelocity);
        SmartDashboard.putNumber("Turret Encoder Reading Position", turretEncoderReadingPosition);
        SmartDashboard.putNumber("Turret Encoder Reading Velocity", turretEncoderReadingVelocity);
        SmartDashboard.putNumber("Turret Hood Encoder reading Position", turretHoodPosition);
        SmartDashboard.putNumber("Turret Hood Encoder Reading Velocity", turretHoodVelocity);

        switch (this.requestedPosition) {
            case TARMAC:
                setPointHood = 230_000;
                setPointShooterPID = 6_700;
                setPointRotation = 0;
                break;
            case FENDER:
                setPointHood = 99_000;
                setPointShooterPID = 5_400;
                setPointRotation = 0;
                break;
            case LAUNCHPAD_CLOSE:
                setPointHood = 0; // to be decided
                setPointShooterPID = 0; // tbd
                setPointRotation = 0; // tbd
                break;
            case LAUNCHPAD_FAR:
                // setPointHood = 300000;
                setPointHood = 0;
                setPointShooterPID = 0; // tbd
                setPointRotation = 200000;
                break;
            case NONE:
                setPointShooterPID = 0;
                setPointHood = 0;
                setPointRotation = 0;
                shooterPid.reset();
                break;
            case RESET:
                setPointShooterPID = 0;
                setPointHood = Double.NEGATIVE_INFINITY;
                setPointRotation = 0;
                shooterMotor.setSelectedSensorPosition(0);
                shooterPid.reset();
                break;
            case TEST:
                setPointShooterPID = 3000;
                setPointHood = 100_000;
                break;
            default:
                break;
        }

        double hoodCurrent = hoodMotor.getStatorCurrent();
        System.out.println("hood setpoint: " + setPointHood + ", current: " + hoodCurrent);
        if (setPointHood == Double.NEGATIVE_INFINITY) {
            hoodMotor.set(ControlMode.PercentOutput, 0);
            hoodReady = false;
            stallCount = 0;
        } else if (setPointHood == 0) {
            hoodReady = false;
            if (hoodCurrent > hoodStallCurrent) {
                stallCount++;
            }
            if (stallCount > stallCountLimit) {
                hoodMotor.set(ControlMode.PercentOutput, 0);
                setShotPosition(ShotPosition.RESET);
                stallCount = 0;
            } else {
                hoodMotor.set(ControlMode.PercentOutput, -0.4);
            }
        } else if(turretHoodPosition >= setPointHood - hoodBuffer && turretHoodPosition <= setPointHood + hoodBuffer){
            hoodMotor.set(ControlMode.PercentOutput, 0);
            hoodReady = true;
            stallCount = 0;
        }else if(turretHoodPosition <= setPointHood - hoodBuffer){
            hoodMotor.set(ControlMode.PercentOutput, hoodSpeed);
            hoodReady = false;
            stallCount = 0;
        }else if (turretHoodPosition >= setPointHood + hoodBuffer){
            hoodMotor.set(ControlMode.PercentOutput, -hoodSpeed);
            hoodReady = false;
            stallCount = 0;
        }

        if (setPointRotation == Double.NEGATIVE_INFINITY) {
            turretMotor.set(ControlMode.PercentOutput, 0);
            turretReady = false;
        } else if(turretEncoderReadingPosition >= setPointRotation - turretBuffer && turretEncoderReadingPosition <= setPointRotation + turretBuffer){
            turretMotor.set(ControlMode.PercentOutput, 0);
            turretReady = true;
        }else if(turretEncoderReadingPosition <= setPointRotation - turretBuffer){
            turretMotor.set(ControlMode.PercentOutput, turretSpeed);
            turretReady = false;
        }else if (turretEncoderReadingPosition >= setPointRotation + turretBuffer){
            turretMotor.set(ControlMode.PercentOutput, -turretSpeed);
            turretReady = false;
        }

        if(setPointShooterPID != 0){
            currentOutput = shooterPid.calculate(shooterEncoderReadingVelocity, setPointShooterPID);
            currentOutput += 0.02; // hack "feed forward"
            currentOutput = Utils.normalizePwm(currentOutput);
            SmartDashboard.putNumber("Setpoint for the shooter is: ", setPointShooterPID);
            SmartDashboard.putNumber("Current Output is: ", shooterEncoderReadingVelocity);
        } else {
            currentOutput = 0;
        }
        this.shooterMotor.set(ControlMode.PercentOutput, currentOutput);
        // System.out.println("Shooter setpoint " + setPointShooterPID + ", speed " + shooterEncoderReadingVelocity + ", power " + currentOutput);
    }

    @Override
    public void findAndCenterTarget() {}

    @Override
    public void centerTarget(double tx){}

    @Override
    public void getDistance(double ty, double angle1, double angle2){}

    @Override
    public boolean blocking() {
        boolean spinningUp = setPointShooterPID > 0 && !shooterPid.atSetpoint();
        // threshold to allow for a little bit of sensor movement around 0, not requiring absolute stillness
        double threshold = 500;
        double absoluteWheelSpeed = Math.abs(shooterEncoderReadingVelocity);
        boolean spinningDown = setPointShooterPID == 0 && absoluteWheelSpeed > threshold;
        return spinningUp || spinningDown;
        // return false;
    }

    @Override
    public boolean readyToShoot(){
        return turretReady && hoodReady && shooterPid.atSetpoint();
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
        this.hoodMotor.setSelectedSensorPosition(0);
    }

    @Override
    public void lifecycleStatusChanged(LifecycleEvent previous, LifecycleEvent current) {
        switch (current) {
            case ON_INIT:
                this.hoodMotor.setSelectedSensorPosition(0);
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
                this.hoodMotor.setSelectedSensorPosition(0);
                break;
        }
    }
}
