package frc.robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;

// SUBSYSTEM IMPORTS
import frc.robot.subsystems.BallPath.Elevator.Elevator;
import frc.robot.subsystems.BallPath.Intake.Intake;
import frc.robot.subsystems.BallPath.Shooter.Shooter;
import frc.robot.subsystems.Drivetrain.Drive;

// import java.util.concurrent.TimeUnit;
// import ca.team3161.lib.robot.TitanBot;

public class Autonomous {

    private Drive drivetrain;
    private Shooter shooter;
    private Intake intake;
    private Elevator elevator;
    private double wheelCircumference = Math.pow((3.14*2), 2);
    private double distance;
    private double setPoint;

    public Autonomous(Drive drivetrain, Shooter shooter, Intake intake, Elevator elevator){
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.intake = intake;
        this.elevator = elevator;
        this.distance = 0;
        this.setPoint = 0;
    }

    double calcDistance(RelativeEncoder encoder){

        distance = calcTicks(encoder) * wheelCircumference;

        return distance;
    }

    double calcTicks(RelativeEncoder encoder){
        // Calculates ticks per revolution(shaft rotation)
        double gearRatio = 8;
        double ticksPer = encoder.getCountsPerRevolution();
        double revs = encoder.getPosition();

        double ticks = ((ticksPer * revs) / gearRatio);

        return ticks;
    }

    double getSetPoint(RelativeEncoder encoder){
        return calcDistance(encoder) / wheelCircumference;
    }

    // void setSetPoint(double targetDistance){
    //     setPoint = targetDistance / wheelCircumference;
    // }

    // Inches to Ticks
    double convertIT(double distance){
        return distance / wheelCircumference;
    }

    // Ticks to Inches
    double convertTI(double ticks){
        return ticks * wheelCircumference;
    }

    void path(double targetDistance){

        /*
        if not at target
        target - current position
        */
    }
    
}