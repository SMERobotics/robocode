package com.technodot.ftc.twentyfive.common;

public class ArtifactInventory {
    public Artifact left;
    public Artifact right;

    public enum Side {
        LEFT,
        RIGHT,
        BOTH,
        NONE
    }

    public ArtifactInventory() {
        left = Artifact.NONE;
        right = Artifact.NONE;
    }

    public void setArtifact(Side side, Artifact artifact) {
        if (side == Side.LEFT || side == Side.BOTH) {
            left = artifact;
        }
        if (side == Side.RIGHT || side == Side.BOTH) {
            right = artifact;
        }
    }

    public Artifact getArtifact(Side side) {
        if (side == Side.LEFT) {
            return left;
        } else if (side == Side.RIGHT) {
            return right;
        }
        return Artifact.NONE;
    }

    public Side findArtifact(Artifact artifact) {
        boolean leftMatch = left == artifact;
        boolean rightMatch = right == artifact;

        if (leftMatch && rightMatch) {
            return Side.BOTH;
        } else if (leftMatch) {
            return Side.LEFT;
        } else if (rightMatch) {
            return Side.RIGHT;
        }
        return Side.NONE;
    }
}
