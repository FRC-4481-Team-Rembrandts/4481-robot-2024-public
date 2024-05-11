package frc.team4481.lib.util;

import edu.wpi.first.wpilibj.Timer;

public class CountingDelay {
    boolean lock;
    double startTimeStamp;

    public CountingDelay() {
        reset();
    }

    public boolean delay(double time){
        if (!lock) {
            startTimeStamp = Timer.getFPGATimestamp();
            lock = true;
        }
        return Timer.getFPGATimestamp() - startTimeStamp >= time;
    }
    public void reset(){
        lock = false;
    }
}
