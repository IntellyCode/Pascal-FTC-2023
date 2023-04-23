package org.firstinspires.ftc.teamcode.Common.Components;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Autonomous.Camera.Pipeline;
import org.firstinspires.ftc.teamcode.Autonomous.Camera.Util.BlueConeDetectionUtil;
import org.firstinspires.ftc.teamcode.Autonomous.Camera.Util.ConeDetectionUtil;
import org.firstinspires.ftc.teamcode.Autonomous.Camera.Util.RedConeDetectionUtil;
import org.firstinspires.ftc.teamcode.Common.RobotTeamColor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;



public class CameraComponent {
    public OpenCvWebcam webcam;

    public Pipeline pipeline;

    int height;
    int width;
    public CameraComponent(HardwareMap hardwareMap, RobotTeamColor color, int width, int height){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        if(color == RobotTeamColor.Blue)
            this.pipeline = new Pipeline(webcam, new BlueConeDetectionUtil());
        else if(color == RobotTeamColor.Red)
            this.pipeline = new Pipeline(webcam, new RedConeDetectionUtil());

        webcam.setPipeline(this.pipeline);
        this.height = height;
        this.width = width;
        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.

    }
    public void start(){
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(width,height, OpenCvCameraRotation.UPRIGHT);

            }
            @Override
            public void onError(int errorCode) {

            }
        });

    }

}
