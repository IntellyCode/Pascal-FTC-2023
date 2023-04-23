package org.firstinspires.ftc.teamcode.Common.Actions;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Common.Components.DriveComponent;
import org.firstinspires.ftc.teamcode.Common.Coordinates;
import org.firstinspires.ftc.teamcode.Common.MotionDirection;
import org.firstinspires.ftc.teamcode.Common.RobotModel;

public class TickMotionAction extends BaseAction {
    double power;
    double displacement;
    MotionDirection direction;
    DriveComponent driveComponent;
    int ticksToPosition;
    Coordinates initialCoordinates;
    public TickMotionAction(RobotModel robotModel, double power, double displacement, MotionDirection direction, Telemetry telemetry){
        super(robotModel,telemetry);
        this.power = power;
        this.displacement=displacement;
        this.direction = direction;
        this.driveComponent = robotModel.getDriveComponent();
    }
    @Override
    public void start(){
        initialCoordinates = robotModel.coordinates;
        if (direction == MotionDirection.vertical){
            robotModel.getDriveComponent().upperLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robotModel.getDriveComponent().lowerLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robotModel.getDriveComponent().upperRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robotModel.getDriveComponent().lowerRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //Calculating the powers
            robotModel.getDriveComponent().lowerLeft.setPower(this.power);
            robotModel.getDriveComponent().upperRight.setPower(this.power);
            robotModel.getDriveComponent().upperLeft.setPower(this.power);
            robotModel.getDriveComponent().lowerRight.setPower(this.power);

            ticksToPosition = (int) (robotModel.getDriveComponent().TICKS_PER_CM * this.displacement);
            robotModel.getDriveComponent().lowerRight.setTargetPosition(ticksToPosition);
            robotModel.getDriveComponent().upperRight.setTargetPosition(ticksToPosition);
            robotModel.getDriveComponent().lowerLeft.setTargetPosition(ticksToPosition);
            robotModel.getDriveComponent().upperLeft.setTargetPosition(ticksToPosition);

            //sets new position
            robotModel.getDriveComponent().upperLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robotModel.getDriveComponent().lowerLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robotModel.getDriveComponent().upperRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robotModel.getDriveComponent().lowerRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            robotModel.getDriveComponent().upperLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robotModel.getDriveComponent().lowerLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robotModel.getDriveComponent().upperRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robotModel.getDriveComponent().lowerRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //Calculating the powers
            robotModel.getDriveComponent().lowerLeft.setPower(power);
            robotModel.getDriveComponent().upperRight.setPower(power);
            robotModel.getDriveComponent().upperLeft.setPower(power);
            robotModel.getDriveComponent().lowerRight.setPower(power);


            ticksToPosition = (int) (robotModel.getDriveComponent().TICKS_PER_CM * this.displacement);
            driveComponent.lowerRight.setTargetPosition(ticksToPosition);
            driveComponent.upperRight.setTargetPosition(-ticksToPosition);
            driveComponent.lowerLeft.setTargetPosition(-ticksToPosition);
            driveComponent.upperLeft.setTargetPosition(ticksToPosition);

            //sets new position
            robotModel.getDriveComponent().upperLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robotModel.getDriveComponent().lowerLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robotModel.getDriveComponent().upperRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robotModel.getDriveComponent().lowerRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    @Override
    public void update() {
        power = getPowerLevel(robotModel.getDriveComponent().lowerRight.getCurrentPosition(), ticksToPosition);
        if (direction == MotionDirection.vertical) {
            robotModel.getDriveComponent().lowerLeft.setPower(power);
            robotModel.getDriveComponent().upperRight.setPower(power);
        } else {
            robotModel.getDriveComponent().lowerLeft.setPower(-power);
            robotModel.getDriveComponent().upperRight.setPower(-power);
        }
        robotModel.getDriveComponent().upperLeft.setPower(power);
        robotModel.getDriveComponent().lowerRight.setPower(power);


        //Coordinates update
        int targetTicks = (int) (robotModel.getDriveComponent().TICKS_PER_CM * displacement);
        int currentTicks = robotModel.getDriveComponent().upperLeft.getCurrentPosition();
        telemetry.addData("currentTicks", currentTicks);
        float cmTraveled = currentTicks / robotModel.getDriveComponent().TICKS_PER_CM;
        telemetry.addData("cmTraveled", cmTraveled);
        //updating model
        if(direction == MotionDirection.vertical) {

            Coordinates vector = new Coordinates(0, 0);
            double radAngle = robotModel.absAngle/180*Math.PI;
            vector = new Coordinates(cmTraveled * Math.cos(radAngle), cmTraveled * Math.sin(radAngle));
            robotModel.coordinates = Coordinates.add(initialCoordinates, vector);
            Coordinates targetCoordinates = Coordinates.add(initialCoordinates, vector);
            robotModel.coordinates = targetCoordinates;
        } else if(direction == MotionDirection.horizontal) {

            Coordinates vector = new Coordinates(0, 0);
            //Horizontal vector is perpendicular to the vertical, so +90 degrees.
            double radAngle = (robotModel.absAngle + 90)/180*Math.PI;
            vector = new Coordinates(-cmTraveled * Math.cos(radAngle), cmTraveled * Math.sin(radAngle));
            robotModel.coordinates = Coordinates.add(initialCoordinates, vector);
            Coordinates targetCoordinates = Coordinates.add(initialCoordinates, vector);
            robotModel.coordinates = targetCoordinates;
        }


        //checking if finished
        if (targetTicks == currentTicks) {
            finished = true;
        }
    }
    double getPowerLevel (int currentTicks, int ticksToPosition){
        double min_power = 0.5; //0.5
        if (this.power < min_power){
            min_power=this.power;
        }
        double max_power = 1; //1
        double tickDifference = ticksToPosition - currentTicks;
        double powerLevel = min_power + (tickDifference * 3 / ticksToPosition) * (max_power - min_power);

        // Make sure the power level is within the valid range of 0.5 to 1.0
        powerLevel = Math.max(powerLevel, min_power);
        powerLevel = Math.min(powerLevel, max_power);

        return powerLevel;
    }
}
