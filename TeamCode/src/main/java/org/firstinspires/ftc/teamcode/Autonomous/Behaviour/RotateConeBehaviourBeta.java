package org.firstinspires.ftc.teamcode.Autonomous.Behaviour;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousEntryPoint;
import org.firstinspires.ftc.teamcode.Autonomous.FieldModel;
import org.firstinspires.ftc.teamcode.Common.Actions.TurnAction;
import org.firstinspires.ftc.teamcode.Common.Coordinates;
import org.firstinspires.ftc.teamcode.Common.RobotModel;
import org.firstinspires.ftc.teamcode.Common.RobotTeamColor;

public class RotateConeBehaviourBeta implements IBehaviour {
    Telemetry telemetry;
    RobotModel robotModel;
    
    FieldModel fieldModel;
    
    TurnAction rotate;
    int quadrant;
    public RotateConeBehaviourBeta(RobotModel robotModel, FieldModel fieldModel, Telemetry telemetry) {
        this.robotModel = robotModel;
        this.telemetry = telemetry;
        this.fieldModel = fieldModel;
    }

    @Override
    public void update() {
        //telemetry.addData("Quadrant",quadrant);
        if (quadrant == 0){
            if (robotModel.coordinates.getX()<3*FieldModel.CM_PER_TILE && robotModel.robotTeamColor == RobotTeamColor.Blue){
                quadrant = compareCoordinates(fieldModel.leftBlueConePile);
            } else if(robotModel.coordinates.getX()>3*FieldModel.CM_PER_TILE && robotModel.robotTeamColor == RobotTeamColor.Blue){
                quadrant = compareCoordinates(fieldModel.rightBlueConePile);
            } else if (robotModel.coordinates.getX()<3*FieldModel.CM_PER_TILE && robotModel.robotTeamColor == RobotTeamColor.Red){
                quadrant = compareCoordinates(fieldModel.leftRedConePile);
            } else {
                quadrant = compareCoordinates(fieldModel.leftRedConePile);
            }
        }
        if (quadrant==1){
            rotate = new TurnAction(robotModel,90,telemetry);
            rotate.start();
        } else if (quadrant==2) {
            rotate = new TurnAction(robotModel, -90, telemetry);
            rotate.start();
        } else if (quadrant==3){
            rotate = new TurnAction(robotModel,180,telemetry);
            rotate.start();
        } else if (quadrant==4) {
            rotate = new TurnAction(robotModel, -180, telemetry);
            rotate.start();
        }
        if (rotate!=null){
            rotate.update();
            if (rotate.isFinished()){
                finishTheBehaviour();
            }
        }

    }
    public int compareCoordinates(Coordinates coneCoordinates){
        if (coneCoordinates.getX() > robotModel.coordinates.getX() && coneCoordinates.getY() > robotModel.coordinates.getY()){
            return 1;
        } else  if (coneCoordinates.getX() < robotModel.coordinates.getX() && coneCoordinates.getY() > robotModel.coordinates.getY()){
            return 2;
        } else  if (coneCoordinates.getX() < robotModel.coordinates.getX() && coneCoordinates.getY() < robotModel.coordinates.getY()){
            return 3;
        } else{
            return 4;
        }
    }
    void finishTheBehaviour() {
        AutonomousEntryPoint.currentBehaviour = new DriveToConeBehaviour(robotModel,telemetry);
    }
}
