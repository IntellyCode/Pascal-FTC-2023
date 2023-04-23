package org.firstinspires.ftc.teamcode.Autonomous.Behaviour;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousEntryPoint;
import org.firstinspires.ftc.teamcode.Autonomous.Camera.Pipeline;
import org.firstinspires.ftc.teamcode.Common.Actions.ClawMotionAction;
import org.firstinspires.ftc.teamcode.Common.Actions.IAction;
import org.firstinspires.ftc.teamcode.Common.Actions.MoveToCoordinatesAction;
import org.firstinspires.ftc.teamcode.Common.RobotModel;

public class ParkingBehaviour implements IBehaviour {
    RobotModel robotModel;
    Telemetry telemetry;
    IAction moveToPosition;

    public ParkingBehaviour(RobotModel robotModel, Telemetry telemetry){
        this.robotModel = robotModel;
        this.telemetry = telemetry;
        this.robotModel.getCameraComponent().webcam.stopRecordingPipeline();
        this.robotModel.getCameraComponent().webcam.stopStreaming();
        Pipeline.parkingPosition = null;


    }
    @Override
    public void update() {
        if(moveToPosition == null) {
            moveToPosition = new MoveToCoordinatesAction(robotModel, robotModel.getParkingCoordinates(), telemetry);
            moveToPosition.start();
        } else if(!moveToPosition.isFinished())
            moveToPosition.update();
        else if(moveToPosition.isFinished()) { //Если робот доехал до парковки, то...
            finishTheBehaviour();
        }
    }

    void finishTheBehaviour() {
        IAction clawAction = new ClawMotionAction(robotModel,false,telemetry);
        clawAction.start();
        //Maybe do some finishing stuff
        //...
        //...
        //...
        //AutonomousEntryPoint.currentBehaviour = new IBehaviour();
    }
}
