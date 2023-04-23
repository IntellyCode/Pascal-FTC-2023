package org.firstinspires.ftc.teamcode.Autonomous.Camera;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.google.zxing.Result;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousEntryPoint;
import org.firstinspires.ftc.teamcode.Autonomous.Behaviour.DriveInitToPoleBehaviour;
import org.firstinspires.ftc.teamcode.Autonomous.Behaviour.RotateTowardsCone;
import org.firstinspires.ftc.teamcode.Autonomous.Behaviour.RotateTowardsPole;
import org.firstinspires.ftc.teamcode.Autonomous.Behaviour.ScanBehaviour;
import org.firstinspires.ftc.teamcode.Autonomous.Behaviour.DriveToConeBehaviour;
import org.firstinspires.ftc.teamcode.Autonomous.Behaviour.DriveToPoleBehaviour;

import org.firstinspires.ftc.teamcode.Autonomous.Camera.Util.ConeDetectionUtil;
import org.firstinspires.ftc.teamcode.Autonomous.Camera.Util.ReadQRCode;
import org.firstinspires.ftc.teamcode.Autonomous.Camera.Util.RelativePosition;
import org.firstinspires.ftc.teamcode.Autonomous.Camera.Util.YellowPoleDetectionUtil;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class Pipeline extends OpenCvPipeline {
    OpenCvWebcam webcam;
    public static FtcDashboard dashboard;
    public static String parkingPosition=null;

    public static double coneArea = 0;
    public static double poleArea = 0;
    public static RelativePosition conePosition;
    public static RelativePosition polePosition;
    public static boolean coneSelected;
    public static boolean poleSelected;

    public static ConeDetectionUtil coneDetectionUtil;

    public Pipeline(OpenCvWebcam webcam, ConeDetectionUtil coneDetectionUtil){
        this.webcam = webcam;
        this.dashboard =FtcDashboard.getInstance();
        this.coneDetectionUtil = coneDetectionUtil;
    }
    @Override
    public Mat processFrame(Mat input) {
        Result result;
        Bitmap bitmap = Bitmap.createBitmap(input.cols(), input.rows(), Bitmap.Config.ARGB_8888);
       if (AutonomousEntryPoint.currentBehaviour instanceof ScanBehaviour) {
            Mat mat = ReadQRCode.process(input);
            mat = ReadQRCode.enhanceSaturationAndContrast(mat);
            dashboard.sendImage(ReadQRCode.matToBitmap(mat));
            result = ReadQRCode.readQRCode(mat);
            if (result!=null){
                parkingPosition = result.getText();
            }
        } else if (AutonomousEntryPoint.currentBehaviour instanceof DriveToConeBehaviour || AutonomousEntryPoint.currentBehaviour instanceof RotateTowardsCone) {
            final Mat extractedColors = coneDetectionUtil.extractColors(input);
            // Process frame
            final Mat processed = coneDetectionUtil.processImage(extractedColors);
            // Mark outer contour
            coneDetectionUtil.markOuterContour(processed, extractedColors);
            Utils.matToBitmap(extractedColors, bitmap);
            dashboard.sendImage(bitmap);
            input.release();
            extractedColors.copyTo(input);
            extractedColors.release();
            processed.release();
        } else if(AutonomousEntryPoint.currentBehaviour instanceof DriveToPoleBehaviour || AutonomousEntryPoint.currentBehaviour instanceof RotateTowardsPole ||AutonomousEntryPoint.currentBehaviour instanceof DriveInitToPoleBehaviour) {
            final Mat extractedColorsYellow = YellowPoleDetectionUtil.extractColors(input);
            // Process frame
            final Mat processedYellow = YellowPoleDetectionUtil.processImage(extractedColorsYellow);
            // Mark outer contour
            YellowPoleDetectionUtil.markOuterContour(processedYellow, extractedColorsYellow);
            Utils.matToBitmap(extractedColorsYellow, bitmap);
            dashboard.sendImage(bitmap);
            input.release();
            extractedColorsYellow.copyTo(input);
            extractedColorsYellow.release();
            processedYellow.release();
        } else{
           Utils.matToBitmap(input, bitmap);
           dashboard.sendImage(bitmap);
       }


        return input;
    }


    @Override
    public void onViewportTapped() {
        this.webcam.pauseViewport();
        /*
        viewportPaused = !viewportPaused;
        if (viewportPaused) {
            this.webcam.pauseViewport();
        } else {
            this.webcam.resumeViewport();
        }*/
    }
}