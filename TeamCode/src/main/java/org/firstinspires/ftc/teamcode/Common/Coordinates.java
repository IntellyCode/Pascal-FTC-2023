package org.firstinspires.ftc.teamcode.Common;

import java.util.List;

public class Coordinates {
    double scaleFactor;
    public double getScaleFactor() {
        return scaleFactor;
    }

    double x;
    public double getX() { return x*scaleFactor; }
    double y;
    public double getY() { return y*scaleFactor; }

    public Coordinates(double x, double y) {
        scaleFactor = 1;
        this.x = x;
        this.y = y;
    }

    public Coordinates(double x, double y, double scaleFactor) {
        this.scaleFactor = scaleFactor;
        this.x = x;
        this.y = y;
    }

    public boolean equals(Coordinates coordinates) {
        if(coordinates.getX() == this.getX() && coordinates.getY() == this.getY())
            return true;
        return false;
    }

    public static Coordinates add(Coordinates coordinates1, Coordinates coordinates2) {
        double newX = coordinates1.getX() + coordinates2.getX();
        double newY = coordinates1.getY() + coordinates2.getY();
        return new Coordinates(newX, newY);
    }

    public static Coordinates findClosestCoordinates(Coordinates targetCoordinates, List<Coordinates> coordinates) {
        Coordinates closestCoordinates = coordinates.get(0);
        coordinates.remove(0);
        for (Coordinates coordinate: coordinates) {
            double distance = Math.abs(targetCoordinates.getX()-coordinate.getX()) +
                              Math.abs(targetCoordinates.getY()-coordinate.getY());
            double previousDistance = Math.abs(targetCoordinates.getX()-closestCoordinates.getX()) +
                                      Math.abs(targetCoordinates.getY()-closestCoordinates.getY());
            if(distance < previousDistance)
                closestCoordinates = coordinate;
        }
        return closestCoordinates;
    }

}
