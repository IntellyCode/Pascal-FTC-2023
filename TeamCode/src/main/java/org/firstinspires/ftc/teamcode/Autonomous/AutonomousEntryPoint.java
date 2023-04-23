package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autonomous.Behaviour.DriveToPoleBehaviour;
import org.firstinspires.ftc.teamcode.Autonomous.Behaviour.IBehaviour;
import org.firstinspires.ftc.teamcode.Autonomous.Behaviour.ParkingBehaviour;
import org.firstinspires.ftc.teamcode.Autonomous.Behaviour.ScanBehaviour;
import org.firstinspires.ftc.teamcode.Autonomous.Camera.Pipeline;
import org.firstinspires.ftc.teamcode.Common.Actions.ClawMotionAction;
import org.firstinspires.ftc.teamcode.Common.Actions.IAction;
import org.firstinspires.ftc.teamcode.Common.Coordinates;
import org.firstinspires.ftc.teamcode.Common.RobotModel;
import org.firstinspires.ftc.teamcode.Common.RobotTeamColor;

public class AutonomousEntryPoint {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    RobotModel robotModel;
    FieldModel fieldModel;

    public static TelemetryPacket telemetryPacket;
    public static IBehaviour currentBehaviour;
    public static ElapsedTime elapsedTime;


    public AutonomousEntryPoint(Coordinates startingCoordinates, RobotTeamColor teamColor, HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        init(startingCoordinates, teamColor);
    }

    void init(Coordinates startingCoordinates, RobotTeamColor teamColor) {
        robotModel = new RobotModel(hardwareMap, teamColor,
               startingCoordinates);
        robotModel.absAngle = 90;
        fieldModel = new FieldModel(startingCoordinates);
        robotModel.getCameraComponent().start();
        IAction clawAction = new ClawMotionAction(robotModel,false,telemetry);
        clawAction.start();
        clawAction = new ClawMotionAction(robotModel,true,telemetry);
        clawAction.start();
        //
        currentBehaviour = new ScanBehaviour(robotModel,fieldModel,telemetry); //scanner
        //this.robotModel.setParkingCoordinates(fieldModel.getThirdParkingPosition());
        //currentBehaviour = new ParkingBehaviour(robotModel,telemetry);
        //currentBehaviour = new ToPoleBehaviour(robotModel,telemetry);
        //currentBehaviour = new ToConeBehaviour(robotModel,telemetry);
        //currentBehaviour = new RotateConeBehaviour(robotModel,fieldModel,telemetry);
        //currentBehaviour = new DriveToPoleBehaviour(robotModel,telemetry);

    }


    public void loop() {
        if (elapsedTime == null){
            elapsedTime = new ElapsedTime();
        }
        currentBehaviour.update();
        telemetry.addData("parkingPosition", Pipeline.parkingPosition);
        telemetry.update();
        //From pipeline
        telemetryPacket = new TelemetryPacket();

        telemetryPacket.put("Current behaviour",currentBehaviour);
        telemetryPacket.put("X",robotModel.coordinates.getX());
        telemetryPacket.put("Y",robotModel.coordinates.getY());
        if (elapsedTime.seconds()>25){
          telemetryPacket.put("Time to park",true);
          if (robotModel.getParkingCoordinates()==null){
              telemetryPacket.put("No parking",true);
              this.robotModel.setParkingCoordinates(fieldModel.getThirdParkingPosition());
              telemetry.addData("Forced parking",true);
              //currentBehaviour = new ParkingBehaviour(robotModel,telemetry);
              Pipeline.parkingPosition ="3";
          }
            if ( !(currentBehaviour instanceof ParkingBehaviour)) currentBehaviour = new ParkingBehaviour(robotModel,telemetry);
         }
        Pipeline.dashboard.sendTelemetryPacket(telemetryPacket);

    }

}
