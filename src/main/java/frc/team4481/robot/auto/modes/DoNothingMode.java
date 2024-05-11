package frc.team4481.robot.auto.modes;

import frc.team4481.lib.auto.actions.WaitAction;
import frc.team4481.lib.auto.mode.AutoModeBase;
import frc.team4481.lib.auto.mode.AutoModeEndedException;
import frc.team4481.robot.auto.selector.AutoMode;
import frc.team4481.robot.auto.selector.Disabled;

/**
 * Default AutoMode
 */

@AutoMode(displayName = "[All] Do nothing")
@Disabled
public class DoNothingMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new WaitAction(0.1));
    }

    /**
     * @throws AutoModeEndedException
     */
    @Override
    protected void initialize() throws AutoModeEndedException {

    }
}
