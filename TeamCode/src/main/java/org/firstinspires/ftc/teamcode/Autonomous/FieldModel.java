package org.firstinspires.ftc.teamcode.Autonomous;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.Common.Coordinates;

import java.util.ArrayList;
import java.util.List;

public class FieldModel {

    public static final double CM_PER_TILE = 61.5;
    double maxHeight;
    public double getMaxHeight() {
        return maxHeight;
    }

    double maxWidth;
    public double getMaxWidth() {
        return maxWidth;
    }


    //Cone related
    List<Coordinates> lowerJunctions;
    public List<Coordinates> getLowerJunctions() {
        return lowerJunctions;
    }

    List<Coordinates> mediumJunctions;
    public List<Coordinates> getMediumJunctions() {
        return mediumJunctions;
    }

    List<Coordinates> higherJunctions;
    public List<Coordinates> getHigherJunctions() {
        return higherJunctions;
    }
    public Coordinates leftRedConePile;
    public Coordinates rightRedConePile;
    public Coordinates leftBlueConePile;
    public  Coordinates rightBlueConePile;
    Coordinates initialPosition;
    public Coordinates getInitialPosition() {
        return initialPosition;
    }
    //Parking related
    Coordinates parkingCone;
    public Coordinates getParkingCone() {
        return parkingCone;
    }
    final Coordinates firstPPVector = new Coordinates(-1, 1, CM_PER_TILE);
    final Coordinates secondPPVector = new Coordinates(0, 1, CM_PER_TILE);
    final Coordinates thirdPPVector = new Coordinates(1, 1, CM_PER_TILE);
    public Coordinates getFirstParkingPosition() {
        return Coordinates.add(initialPosition, firstPPVector);
    }
    public Coordinates getSecondParkingPosition() {
        return Coordinates.add(initialPosition, secondPPVector);
    }
    public Coordinates getThirdParkingPosition() {
        return Coordinates.add(initialPosition, thirdPPVector);
    }
    public FieldModel(Coordinates initialPosition) {
        this.initialPosition = initialPosition;

        maxHeight = 6*CM_PER_TILE;
        maxWidth = 6*CM_PER_TILE;
        lowerJunctions = new ArrayList<>();
        lowerJunctions.add(new Coordinates(1, 2, CM_PER_TILE));
        lowerJunctions.add(new Coordinates(2, 1, CM_PER_TILE));
        lowerJunctions.add(new Coordinates(1, 4, CM_PER_TILE));
        lowerJunctions.add(new Coordinates(4, 1, CM_PER_TILE));
        lowerJunctions.add(new Coordinates(2, 5, CM_PER_TILE));
        lowerJunctions.add(new Coordinates(4, 5, CM_PER_TILE));
        lowerJunctions.add(new Coordinates(5, 4, CM_PER_TILE));
        lowerJunctions.add(new Coordinates(5, 2, CM_PER_TILE));

        mediumJunctions = new ArrayList<>();
        mediumJunctions.add(new Coordinates(2, 2, CM_PER_TILE));
        mediumJunctions.add(new Coordinates(2, 4, CM_PER_TILE));
        mediumJunctions.add(new Coordinates(4, 4, CM_PER_TILE));
        mediumJunctions.add(new Coordinates(4, 2, CM_PER_TILE));

        higherJunctions = new ArrayList<>();
        higherJunctions.add(new Coordinates(3, 2, CM_PER_TILE));
        higherJunctions.add(new Coordinates(2, 3, CM_PER_TILE));
        higherJunctions.add(new Coordinates(3, 4, CM_PER_TILE));
        higherJunctions.add(new Coordinates(4, 3, CM_PER_TILE));

        leftRedConePile =new Coordinates(0,2.5,CM_PER_TILE);
        rightRedConePile =new Coordinates(6,2.5,CM_PER_TILE);
        leftBlueConePile =new Coordinates(0,2.5,CM_PER_TILE);
        rightBlueConePile =new Coordinates(6,2.5,CM_PER_TILE);

    }
}
