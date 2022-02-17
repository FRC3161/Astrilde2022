package frc.robot.subsystems.BallPath;

import java.util.concurrent.TimeUnit;

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;

import frc.robot.subsystems.BallPath.Intake.Intake;
import frc.robot.subsystems.BallPath.Elevator.Elevator;
import frc.robot.subsystems.BallPath.Shooter.Shooter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BallPathImpl extends RepeatingPooledSubsystem implements BallPath {

    private final Intake intake;
    private final Elevator elevator;
    private final Ultrasonic elevatorSensor;
    private final Shooter shooter;
    private String action;
    private double horizontalDistanceToTarget;

    private long intakeStartedTime = -1;

    public BallPathImpl(Intake intake, Elevator elevator, Shooter shooter, Ultrasonic elevatorSensor) {
        super(20, TimeUnit.MILLISECONDS);
        this.intake = intake;
        this.elevator = elevator;
        this.elevatorSensor = elevatorSensor;
        this.shooter = shooter;
        this.action = "IDLE";
        this.horizontalDistanceToTarget = 0;
    }

    @Override
    public void defineResources(){}

    @Override
    public void task() {
        if (this.action.equals("START_INTAKE")){
            if (this.intake.checkColour() && !this.checkIfPrimed()){ 
                long now = System.nanoTime(); // current unix timestamp in nanoseconds 
                if (this.intakeStartedTime < 0) { 
                    this.intakeStartedTime = now; 
                } 
                this.intake.start(); 
                if (now > this.intakeStartedTime + TimeUnit.SECONDS.toNanos(3)) { 
                    this.intake.stop(); 
                    this.intakeStartedTime = -1; 
                    // we have completed this action and don't want to repeat it all over again, 
                    // so maybe we do this too: 
                    this.action = "IDLE"; 
                } 
            }
        }

        // Limelight data collection.
        // Obtain values from the limelight.
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
        NetworkTableEntry tv = table.getEntry("tv");

        // Read values periodically
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        double v = tv.getDouble(0.0);

        // Post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
        SmartDashboard.putNumber("LimelightTargetInSight", v);

        // Constantly calculate the (distance?) and centre the shooter (given that the angle is less than 60 degrees).
        if (Math.abs(x) <= 60) {
            // Centre the shooter with the target, calculate the horizontal distance to the said target, and set the hood angle (in preparation for shooting) according to the horizontal distance.
            this.findAndCenterTarget(x);
            this.horizontalDistanceToTarget = shooter.getDistance(y);
            this.shooter.setHoodAngle(horizontalDistanceToTarget);

            // If the shooter is ready to shoot.
            if (this.readyToShoot()) {
                this.startShooter(horizontalDistanceToTarget);  // The distance to the target was added; prior to such the method was simply called without a required argument.
            } else {
                this.stopShooter();
            }
        } else {
            this.stopShooter();
        }
    }

    // Declare interface with team
    @Override
    public void startIntake(){
        this.intake.start();
        this.action = "START_INTAKE";
    }

    @Override
    public void reverseIntake(){
        this.intake.reverse();
    }

    @Override
    public void stopIntake(){
        this.intake.stop();
    }

    @Override
    public boolean checkIfPrimed(){
        // if (ballUnderElevator){
        //     return true;
        // }
        if (this.elevatorSensor.getRangeInches() <= 4 ){
            return true;
        }
        return false;
    }

    @Override
    public void startElevator(){
        this.elevator.start();
    }
    
    // can be used to stop a ball from going up the elevator in the event that we cannot shoot the ball
    @Override
    public void reverseElevator(){
        this.elevator.reverse();
    }

    @Override
    public void stopElevator(){
        this.elevator.stop();
    }

    @Override
    public void findAndCenterTarget(double tx) {
        this.shooter.centerTarget(tx);
    }

    @Override
    public boolean readyToShoot(){
        return this.shooter.readyToShoot();
    }

    @Override
    public void startShooter(double distance) {
        this.shooter.start(distance);
    }

    @Override
    public void stopShooter(){
        this.shooter.stop();
    }
        
    @Override
    public void lifecycleStatusChanged(LifecycleEvent previous, LifecycleEvent current) {}
   
}
