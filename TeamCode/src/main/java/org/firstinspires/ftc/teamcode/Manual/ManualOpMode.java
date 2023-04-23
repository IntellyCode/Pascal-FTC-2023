package org.firstinspires.ftc.teamcode.Manual;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Common.RobotTeamColor;
import org.firstinspires.ftc.teamcode.Common.RotationDirection;
import org.firstinspires.ftc.teamcode.Common.Actions.VibrateAction;
import org.firstinspires.ftc.teamcode.Common.RobotModel;
import org.firstinspires.ftc.teamcode.Common.Components.ArmPosition;

@TeleOp
public class ManualOpMode extends LinearOpMode {
    RobotModel model;
    ManualDrive driveController;
    ManualArm armController;

    boolean liftActionOccurred;

    boolean firstIteration = true;
    public static RotationDirection rot =RotationDirection.none;
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        while(true) {
            driveControl();
            armControl();
            telemetry.update();
        }
    }
    void initialize() {
        model = new RobotModel(hardwareMap, RobotTeamColor.Blue, null);
        armController = new ManualArm(model);
        driveController = new ManualDrive(model, telemetry);
    }
    void driveControl() {
        {
            double turn = gamepad1.right_stick_x;
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            driveController.setTurn(turn);
            driveController.setX(x);
            driveController.setY(y);
        }
        if (gamepad1.left_bumper && rot==RotationDirection.none){
            driveController.startTurning(-90);
            rot = RotationDirection.left;
            telemetry.addData("Pressed",rot);
        } else if (gamepad1.right_bumper && rot== RotationDirection.none){
            driveController.startTurning( 90);
            rot = RotationDirection.right;
            telemetry.addData("Pressed",rot);
        }
        if (rot!=RotationDirection.none ){
            rot = driveController.continueTurning();
        }
        telemetry.addData("Pressed",rot);
        driveController.update();
    }

    void armControl() {
        telemetry.addData("Busy",liftActionOccurred);

        VibrateAction vibrateController = new VibrateAction(model,telemetry);
        if (gamepad2.left_trigger > 0.1f){
            armController.controlLiftManually(-1);
            liftActionOccurred = true;

        }
        else if (gamepad2.right_trigger > 0.1f){
            armController.controlLiftManually(1);
            liftActionOccurred = true;

        } else if (gamepad2.right_trigger < 0.1f ||gamepad2.left_trigger < 0.1f){
            armController.controlLiftManually(0);
            liftActionOccurred = false;

        }
        else if(gamepad2.circle) {
            armController.liftArmToPosition(ArmPosition.Zero);


        }
        else if(gamepad2.triangle) {
            armController.liftArmToPosition(ArmPosition.First);

        }
        else if(gamepad2.square) {
            armController.liftArmToPosition(ArmPosition.Second);

        }
        else if(gamepad2.cross){
            armController.liftArmToPosition(ArmPosition.Third);

        }

        if(gamepad2.left_bumper) { //Тут ставишь куда ты хочешь жать для контроля клешни вместо true и false
            armController.setCleshnja(false);

        }else if(gamepad2.right_bumper) {
            armController.setCleshnja(true);

        }
        if (!liftActionOccurred){
            vibrateController.update();
            if (model.vibrated &&firstIteration){
                gamepad2.rumble(500);
                gamepad1.rumble(500);
                firstIteration=false;
            }
        }
        if (!model.vibrated){
            firstIteration= true;
        }
    }
}
