package org.firstinspires.ftc.teamcode.Autonomous.Behaviour;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousEntryPoint;
import org.firstinspires.ftc.teamcode.Autonomous.Camera.Pipeline;
import org.firstinspires.ftc.teamcode.Autonomous.Camera.Util.RelativePosition;
import org.firstinspires.ftc.teamcode.Common.Actions.ClawMotionAction;
import org.firstinspires.ftc.teamcode.Common.Actions.IAction;
import org.firstinspires.ftc.teamcode.Common.Actions.LiftArmAction;
import org.firstinspires.ftc.teamcode.Common.Actions.PowerMotionAction;
import org.firstinspires.ftc.teamcode.Common.Actions.VibrateAction;
import org.firstinspires.ftc.teamcode.Common.MotionDirection;
import org.firstinspires.ftc.teamcode.Common.RobotModel;

public class DriveToConeBehaviour implements IBehaviour {
    //Мне кажется, тут старая версия. Попросить новую.

    Telemetry telemetry;
    RobotModel robotModel;
    double power = 0.5;
    LiftArmAction lift;
    PowerMotionAction powerMotionAction;
    ClawMotionAction clawMotionAction;

    IAction currentAction;

    public DriveToConeBehaviour(RobotModel robotModel, Telemetry telemetry){
        this.robotModel = robotModel;
        this.telemetry = telemetry;
    }
    public void update(){
        if (Pipeline.conePosition== RelativePosition.center){
            currentAction = new VibrateAction(robotModel,telemetry);
            currentAction.start();
            if (robotModel.vibrated&&!robotModel.withCone){
                clawMotionAction = new ClawMotionAction(robotModel,false,telemetry);
                clawMotionAction.start();
                powerMotionAction = new PowerMotionAction(robotModel, 0,MotionDirection.vertical, telemetry);
                finishTheBehaviour();
            }else{powerMotionAction = new PowerMotionAction(robotModel, power, MotionDirection.vertical,telemetry);
            }
            currentAction.update();
        } else if(Pipeline.conePosition == RelativePosition.right){
            powerMotionAction = new PowerMotionAction(robotModel,power,MotionDirection.horizontal,telemetry);
        } else{
            powerMotionAction = new PowerMotionAction(robotModel,-power,MotionDirection.horizontal,telemetry);
        }
        powerMotionAction.update();

    }

    void finishTheBehaviour() {
        this.robotModel.withCone=true;
        AutonomousEntryPoint.currentBehaviour = new RotateTowardsPole(robotModel, telemetry);
    }
}
