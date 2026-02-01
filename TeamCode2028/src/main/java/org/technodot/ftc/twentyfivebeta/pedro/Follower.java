package org.technodot.ftc.twentyfivebeta.pedro;

import com.pedropathing.Drivetrain;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizer;
import com.pedropathing.paths.PathConstraints;

public class Follower extends com.pedropathing.follower.Follower {
    public Follower(FollowerConstants constants, Localizer localizer, Drivetrain drivetrain, PathConstraints pathConstraints) {
        super(constants, localizer, drivetrain, pathConstraints);
    }

    /**
     * This returns if the Follower is currently NOT or DONE following a Path or a PathChain
     * @return returns if the Follower is ready.
     */
    public boolean isReady() {
        return !this.isBusy();
    }

    /**
     * This returns if the Follower if currently at the end of the current Path, without any correction
     * Allows for seamless transitions in between distinct Paths, very different from isReady()
     * @return returns if the Follower is transitionable
     */
    public boolean isTransitionable() {
        return this.atParametricEnd() && this.getHeadingError() < this.getCurrentPath().getPathEndHeadingConstraint();
    }
}
