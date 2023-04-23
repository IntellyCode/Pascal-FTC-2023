package org.firstinspires.ftc.teamcode.Common.Actions;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousEntryPoint;
import org.firstinspires.ftc.teamcode.Common.Components.DriveComponent;
import org.firstinspires.ftc.teamcode.Common.Coordinates;
import org.firstinspires.ftc.teamcode.Common.MotionDirection;
import org.firstinspires.ftc.teamcode.Common.RobotModel;


public class PowerMotionAction extends BaseAction {
    public double power;
    public MotionDirection direction;
    Coordinates initialCoordinates;
    DriveComponent motors;
    public PowerMotionAction(RobotModel robotModel, double power, MotionDirection direction, Telemetry telemetry){
        super(robotModel,telemetry);
        this.power = power;
        this.direction = direction;
        this.initialCoordinates = robotModel.coordinates;
        motors = robotModel.getDriveComponent();
        motors.upperLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.lowerLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.upperRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.lowerRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void start(){
        motors.upperLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.lowerLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.upperRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.lowerRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (direction == MotionDirection.vertical){
            motors.lowerLeft.setPower(power);
            motors.upperRight.setPower(power);
            motors.upperLeft.setPower(power);
            motors.lowerRight.setPower(power);
        } else if (direction == MotionDirection.horizontal){
            motors.lowerLeft.setPower(-power);
            motors.upperRight.setPower(-power);
            motors.upperLeft.setPower(power);
            motors.lowerRight.setPower(power);
        } else{
            motors.lowerLeft.setPower(0);
            motors.upperRight.setPower(0);
            motors.upperLeft.setPower(0);
            motors.lowerRight.setPower(0);
        }
        motors.upperLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors.lowerLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors.upperRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors.lowerRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    @Override
    public void update(){
        //Coordinates update
        int currentTicks = robotModel.getDriveComponent().upperLeft.getCurrentPosition();
        float cmTraveled = currentTicks / robotModel.getDriveComponent().TICKS_PER_CM;
        telemetry.addData("cmtraveled", cmTraveled);
        if (power!=0) {
            if (direction == MotionDirection.vertical) {
                //updating model
                Coordinates vector = new Coordinates(0, 0);
                //Horizontal vector is perpendicular to the vertical, so +90 degrees.
                double radAngle = (robotModel.absAngle) / 180 * Math.PI;
                vector = new Coordinates(cmTraveled * Math.cos(radAngle), cmTraveled * Math.sin(radAngle));
                robotModel.coordinates = Coordinates.add(initialCoordinates, vector);
                Coordinates targetCoordinates = Coordinates.add(initialCoordinates, vector);
                robotModel.coordinates = targetCoordinates;
            } else if (direction == MotionDirection.horizontal) {
                //updating model
                Coordinates vector = new Coordinates(0, 0);
                //Horizontal vector is perpendicular to the vertical, so +90 degrees.
                double radAngle = (robotModel.absAngle + 90) / 180 * Math.PI;
                telemetry.addData("radANGLE", radAngle);
                vector = new Coordinates(cmTraveled * Math.cos(radAngle), cmTraveled * Math.sin(radAngle));
                telemetry.addData("vectorX", vector.getX());
                robotModel.coordinates = Coordinates.add(initialCoordinates, vector);
                Coordinates targetCoordinates = Coordinates.add(initialCoordinates, vector);
                robotModel.coordinates = targetCoordinates;
            }
        }
    }
}
