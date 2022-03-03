package frc.robot.subsystems.BallPath;

import ca.team3161.lib.robot.LifecycleListener;
import ca.team3161.lib.robot.subsystem.Subsystem;

public interface BallPath extends Subsystem, LifecycleListener {
    void setAction(BallAction action);

    enum BallAction {
        NONE,
        FEED,
        SHOOT,
        ;
    }
}
