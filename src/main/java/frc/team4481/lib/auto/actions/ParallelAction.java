package frc.team4481.lib.auto.actions;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


/**
 * Composite action, running all sub-actions at the same time. All actions are started then periodically updated until all actions
 * report being done.
 *
 * @author Team 254 - The Cheesy Poofs
 */
public class ParallelAction implements Action {

    private final ArrayList<Action> actions;

    public ParallelAction(List<Action> actions) {
        this.actions = new ArrayList<>(actions);
    }

    public ParallelAction(Action... actions) {
        this(Arrays.asList(actions));
    }

    @Override
    public boolean isFinished() {
        boolean finish = true;
        ArrayList<Action> removeList = new ArrayList<Action>();
        for (Action action : actions) {
            if (!action.isFinished()) {
                finish = false;
            } else {
                action.done();
                removeList.add(action);
            }
        }
        actions.removeAll(removeList);
        return finish;
    }

    @Override
    public void update() {
        for (Action action : actions) {
            if (!action.isFinished()){
                action.update();
            }
        }
    }

    @Override
    public void done() {
        for (Action action : actions) {
            action.done();
        }
    }

    @Override
    public void start() {
        for (Action action : actions) {
            action.start();
        }
    }
}

// Copyright (c) 2019 Team 254