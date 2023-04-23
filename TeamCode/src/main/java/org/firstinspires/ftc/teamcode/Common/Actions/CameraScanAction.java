package org.firstinspires.ftc.teamcode.Common.Actions;

import com.google.zxing.Result;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autonomous.FieldModel;
import org.firstinspires.ftc.teamcode.Common.Components.CameraComponent;
import org.firstinspires.ftc.teamcode.Common.Coordinates;
import org.firstinspires.ftc.teamcode.Common.RobotModel;

public class CameraScanAction extends BaseAction {
    CameraComponent cameraComponent;
    FieldModel fieldModel;
    Coordinates parkingPosition;
    String position;

    public CameraScanAction(RobotModel robotModel, String position, Telemetry telemetry) {
        super(robotModel, telemetry);
        this.cameraComponent = robotModel.getCameraComponent();
        this.position = position;
    }

    @Override
    public void start() {
            switch (position) {
                case "1":
                    this.robotModel.setParkingCoordinates(fieldModel.getFirstParkingPosition());
                    break;
                case "2":
                    this.robotModel.setParkingCoordinates(fieldModel.getSecondParkingPosition());
                    break;
                case "3":
                    this.robotModel.setParkingCoordinates(fieldModel.getThirdParkingPosition());
                    break;
            }
            robotModel.setParkingCoordinates(parkingPosition);
            finished = true;
    }

    @Override
    public void update() {

    }
}
