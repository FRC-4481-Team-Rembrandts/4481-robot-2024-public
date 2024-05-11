package frc.team4481.lib.hardware;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class LazySolenoid extends Solenoid {
    private boolean last = false;

    public LazySolenoid(PneumaticsModuleType moduleType, int channel) {
        super(moduleType, channel);
    }

    public LazySolenoid(int channel) {
        super(PneumaticsModuleType.REVPH, channel);
    }

    /**
     * Function for simply latching the solenoid state
     *
     * @param latch boolean value to latch solenoid
     */
    public void latch(boolean latch) {
        // TODO TEST
        if (latch && !last) {
            super.set(!super.get());
        }
        last = latch;
    }
}
