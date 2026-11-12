package org.technodot.ftc.twentyfivebeta.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.Localizer;
import com.pedropathing.paths.Path;
import java.util.ArrayList;
import java.util.List;

/** Team helpers around the Pedro 3 follower. */
public class Follower extends com.pedropathing.follower.Follower {
    private final Foresight foresight;
    private final List<Callback> callbacks = new ArrayList<>();

    public Follower(Localizer localizer, Drivetrain drivetrain, Foresight foresight) {
        super(localizer, drivetrain, foresight);
        this.foresight = foresight;
    }

    public boolean isReady() { return !isBusy(); }

    public boolean isTransitionable() {
        return currentPath() != null && atParametricEnd()
                && Math.abs(foresight.headingError()) < foresight.config.headingConstraint.get();
    }

    public void followPath(Path path) { followPath(path, 1, true); }

    public void followPath(Path path, double power, boolean hold) {
        holdEnd.set(hold);
        follow(path.with(foresight.config.maxPathSpeed.at(power)));
        callbacks.clear();
    }

    public void onParametric(double threshold, Runnable action) {
        callbacks.add(new Callback(threshold, action));
    }

    @Override public void update(double deltaTime) {
        super.update(deltaTime);
        for (Callback callback : new ArrayList<>(callbacks)) {
            if (parametricCompletion() >= callback.threshold) {
                callbacks.remove(callback);
                callback.action.run();
            }
        }
    }

    @Override public void stop() {
        super.stop();
        callbacks.clear();
        drivetrain.stop();
    }

    private static class Callback {
        final double threshold;
        final Runnable action;
        Callback(double threshold, Runnable action) {
            this.threshold = threshold;
            this.action = action;
        }
    }
}
