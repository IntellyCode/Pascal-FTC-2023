package org.firstinspires.ftc.teamcode.Autonomous.Behaviour;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousEntryPoint;
import org.firstinspires.ftc.teamcode.Autonomous.Camera.Pipeline;
import org.firstinspires.ftc.teamcode.Autonomous.Camera.Util.RelativePosition;
import org.firstinspires.ftc.teamcode.Autonomous.FieldModel;
import org.firstinspires.ftc.teamcode.Common.Actions.ClawMotionAction;
import org.firstinspires.ftc.teamcode.Common.Actions.IAction;
import org.firstinspires.ftc.teamcode.Common.Actions.LiftArmAction;
import org.firstinspires.ftc.teamcode.Common.Actions.PowerMotionAction;
import org.firstinspires.ftc.teamcode.Common.MotionDirection;
import org.firstinspires.ftc.teamcode.Common.RobotModel;

public class DriveToPoleBehaviour implements IBehaviour{
    Telemetry telemetry;
    RobotModel robotModel;
    PowerMotionAction powerMotionAction;
    final double power = 0.35;
    public DriveToPoleBehaviour(RobotModel robotModel, Telemetry telemetry) {
        this.robotModel = robotModel;
        this.telemetry = telemetry;
        //powerMotionAction = new PowerMotionAction(robotModel,0, MotionDirection.none,telemetry);
    }
    @Override
    public void update() {
            //Object detection
            if (Pipeline.polePosition== RelativePosition.center){

                    powerMotionAction = new PowerMotionAction(robotModel,power, MotionDirection.vertical,telemetry);


                 if(Pipeline.poleArea > 2000){

                     powerMotionAction = new PowerMotionAction(robotModel,0, MotionDirection.vertical,telemetry);


                    finishTheBehaviour();
                }
            } else if(Pipeline.polePosition == RelativePosition.right){

                    powerMotionAction = new PowerMotionAction(robotModel,power, MotionDirection.horizontal,telemetry);


            } else if(Pipeline.polePosition == RelativePosition.left){

                    powerMotionAction = new PowerMotionAction(robotModel,-power, MotionDirection.horizontal,telemetry);

            }
            if (powerMotionAction!=null){
            powerMotionAction.start();
            powerMotionAction.update();
        }
        }
    void finishTheBehaviour() {
       // AutonomousEntryPoint.currentBehaviour = new ParkingBehaviour(robotModel,telemetry);
        AutonomousEntryPoint.currentBehaviour = new PlaceConeBehaviour(this.robotModel,telemetry);
    }
}
