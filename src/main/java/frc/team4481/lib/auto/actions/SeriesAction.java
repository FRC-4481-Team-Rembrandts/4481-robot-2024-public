package frc.team4481.lib.auto.actions;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Executes one action at a time. Useful as a member of {@link ParallelAction}
 *
 * @author Team 254 - The Cheesy Poofs
 */
public class SeriesAction implements Action {
    private Action currentAction;
    private final ArrayList<Action> remainingActions;

    public SeriesAction(List<Action> pActions) {
        remainingActions = new ArrayList<>(pActions);
        currentAction = null;
    }

    public SeriesAction(Action... pActions) {
        this(Arrays.asList(pActions));
    }

    @Override
    public void start() {}

    @Override
    public void update() {
        if (currentAction == null) {
            if (remainingActions.isEmpty()) {
                return;
            }

            currentAction = remainingActions.remove(0);
            currentAction.start();
        }

        currentAction.update();

        if (currentAction.isFinished()) {
            currentAction.done();
            currentAction = null;
        }
    }

    @Override
    public boolean isFinished() {
        return remainingActions.isEmpty() && currentAction == null;
    }

    @Override
    public void done() {}
}

// Copyright (c) 2019 Team 254
