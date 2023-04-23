package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.Autonomous.Camera.Util.ConeDetectionUtil;
import org.firstinspires.ftc.teamcode.Common.Components.ArmComponent;
import org.firstinspires.ftc.teamcode.Common.Components.CameraComponent;
import org.firstinspires.ftc.teamcode.Common.Components.DriveComponent;
import org.firstinspires.ftc.teamcode.Common.Components.SensorComponent;

public class RobotModel {
    //Robot components
    CameraComponent cameraComponent;
    public CameraComponent getCameraComponent() {
        return cameraComponent;
    }
    DriveComponent driveComponent;
    public DriveComponent getDriveComponent() {
        return driveComponent;
    }

    ArmComponent armComponent;
    public ArmComponent getArmComponent() {
        return armComponent;
    }

    SensorComponent sensorComponent;
    public SensorComponent getSensorComponent(){
        return sensorComponent;
    }
    //absolute coordinates on the field
    public Coordinates coordinates;
    public final double HEIGHT = 37;
    public final double WIDTH = 35;

    //angle to the x axis
    public double absAngle;
    //current arm position
    public double armPosition;
    //current grabber state
    public boolean grabberOpen; //for manual

    public boolean withCone;
    Coordinates parkingCoordinates;

    public boolean vibrated = false;

    public RobotTeamColor robotTeamColor;
    public Coordinates getParkingCoordinates() {
        return parkingCoordinates;
    }
    public void setParkingCoordinates(Coordinates parkingCoordinates) {
        this.parkingCoordinates = parkingCoordinates;
    }

    public RobotModel(HardwareMap hardwareMap, RobotTeamColor robotTeamColor, Coordinates initialCoordinates) {
        this.driveComponent = new DriveComponent(hardwareMap);
        this.armComponent = new ArmComponent(hardwareMap);
        this.sensorComponent = new SensorComponent(hardwareMap);
        this.cameraComponent = new CameraComponent(hardwareMap, robotTeamColor,640,480);
        this.armPosition = 0;
        this.robotTeamColor = robotTeamColor;
        coordinates = initialCoordinates;
        absAngle = 90;
    }
}
