package frc.robot.subsystems.BallPath.Elevator;

import ca.team3161.lib.robot.LifecycleListener;
import ca.team3161.lib.robot.subsystem.Subsystem;

public interface Elevator extends Subsystem, LifecycleListener {
    void setAction(ElevatorAction action);
    boolean ballPrimed();

    enum ElevatorAction {
        NONE,
        PRIME,
        REJECT,
        FEED,
        ;
    }
}
