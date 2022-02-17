package frc.robot.subsystems.BallPath;

import ca.team3161.lib.robot.LifecycleListener;
import ca.team3161.lib.robot.subsystem.Subsystem;

public interface BallPath extends Subsystem, LifecycleListener {
    // Declare interface with team
    void startIntake();
    void reverseIntake();
    void stopIntake();
    boolean checkIfPrimed();
    // can be used to stop a ball from going up the elevator in the event that we cannot shoot the ball
    void startElevator();
    void reverseElevator();
    void stopElevator();
    void findAndCenterTarget(double tx);
    boolean readyToShoot();
    void startShooter(double distance);
    void stopShooter();
}
