package org.firstinspires.ftc.teamcode.Autonomous.Behaviour;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousEntryPoint;
import org.firstinspires.ftc.teamcode.Autonomous.FieldModel;
import org.firstinspires.ftc.teamcode.Common.Actions.ClawMotionAction;
import org.firstinspires.ftc.teamcode.Common.Actions.IAction;
import org.firstinspires.ftc.teamcode.Common.Actions.LiftArmAction;
import org.firstinspires.ftc.teamcode.Common.Actions.TickMotionAction;
import org.firstinspires.ftc.teamcode.Common.Components.ArmComponent;
import org.firstinspires.ftc.teamcode.Common.MotionDirection;
import org.firstinspires.ftc.teamcode.Common.RobotModel;

import java.util.LinkedList;
import java.util.Queue;

public class PlaceConeBehaviour implements IBehaviour{
    Queue<IAction> actionSequence;
    IAction currentAction;
    Telemetry telemetry;
    RobotModel robotModel;
    FieldModel fieldModel;

    final double power = 0.5;

    public PlaceConeBehaviour(RobotModel robotModel, Telemetry telemetry) {
        this.robotModel = robotModel;
        this.telemetry = telemetry;

        actionSequence = new LinkedList<IAction>();
        initActionSequence();

        currentAction = actionSequence.poll();
        currentAction.start();
    }

    void initActionSequence() {
        actionSequence.add(new LiftArmAction(robotModel,ArmComponent.PALKA_3_CM,telemetry));
        actionSequence.add(new TickMotionAction(robotModel,0.5,20,MotionDirection.vertical,telemetry));
        actionSequence.add(new LiftArmAction(robotModel, ArmComponent.PALKA_2_CM,telemetry));
        actionSequence.add(new ClawMotionAction(robotModel,false,telemetry));
        actionSequence.add(new TickMotionAction(robotModel,0.5,-19,MotionDirection.vertical,telemetry));
        actionSequence.add(new LiftArmAction(robotModel, 0,telemetry));
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
        this.robotModel.withCone =false;
       // AutonomousEntryPoint.currentBehaviour = new RotateTowardsCone(this.robotModel,telemetry);
    }
}
