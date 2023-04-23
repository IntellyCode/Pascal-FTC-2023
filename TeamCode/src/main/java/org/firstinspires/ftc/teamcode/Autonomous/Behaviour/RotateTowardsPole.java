package org.firstinspires.ftc.teamcode.Autonomous.Behaviour;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousEntryPoint;
import org.firstinspires.ftc.teamcode.Autonomous.FieldModel;
import org.firstinspires.ftc.teamcode.Common.Actions.IAction;
import org.firstinspires.ftc.teamcode.Common.Actions.LiftArmAction;
import org.firstinspires.ftc.teamcode.Common.Actions.MoveToCoordinatesAction;
import org.firstinspires.ftc.teamcode.Common.Actions.TickMotionAction;
import org.firstinspires.ftc.teamcode.Common.Actions.TurnAction;
import org.firstinspires.ftc.teamcode.Common.Coordinates;
import org.firstinspires.ftc.teamcode.Common.MotionDirection;
import org.firstinspires.ftc.teamcode.Common.RobotModel;

import java.util.LinkedList;
import java.util.Queue;

public class RotateTowardsPole implements IBehaviour{
    Queue<IAction> actionSequence;
    IAction currentAction;
    Telemetry telemetry;
    RobotModel robotModel;
    public RotateTowardsPole(RobotModel robotModel, Telemetry telemetry) {
        this.robotModel = robotModel;
        this.telemetry = telemetry;
        actionSequence = new LinkedList<>();

        initActionSequence();
        currentAction = actionSequence.poll();
        currentAction.start();
    }
    public void initActionSequence(){
        actionSequence.add(new TurnAction(robotModel,-180,telemetry));
        actionSequence.add(new TickMotionAction(robotModel,1,60, MotionDirection.vertical,telemetry));
    }
    @Override
    public void update() {
        if(currentAction.isFinished()) {
            if(actionSequence.peek() != null) {
                currentAction = actionSequence.poll();
                currentAction.start();
            } else {
                finishTheBehaviour();
            }
        } else {
            currentAction.update();
        }
    }

    void finishTheBehaviour() {
        AutonomousEntryPoint.currentBehaviour = new DriveToConeBehaviour(robotModel,telemetry);
    }
}
