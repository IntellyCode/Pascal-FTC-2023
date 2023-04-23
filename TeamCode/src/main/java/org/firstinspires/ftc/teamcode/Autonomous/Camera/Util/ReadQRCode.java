package org.firstinspires.ftc.teamcode.Autonomous.Camera.Util;

import android.graphics.Bitmap;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.MultiFormatReader;
import com.google.zxing.NotFoundException;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.Result;
import com.google.zxing.common.HybridBinarizer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class ReadQRCode {
    public static Mat enhanceSaturationAndContrast(Mat frame) {
        // Convert the image to HSV color space
        Mat hsvImage = new Mat();
        Imgproc.cvtColor(frame, hsvImage, Imgproc.COLOR_BGR2HSV);

        // Split the HSV channels
        List<Mat> hsvChannels = new ArrayList<>();
        Core.split(hsvImage, hsvChannels);

        // Enhance saturation
        double saturationScale = 1.2; // Adjust this value to control the saturation enhancement
        Core.multiply(hsvChannels.get(1), new Scalar(saturationScale), hsvChannels.get(1));

        // Enhance value (brightness)
        double valueScale = 1.2; // Adjust this value to control the brightness enhancement
        Core.multiply(hsvChannels.get(2), new Scalar(valueScale), hsvChannels.get(2));

        // Merge the enhanced HSV channels
        Core.merge(hsvChannels, hsvImage);

        // Convert the image back to BGR color space
        Mat bgrImage = new Mat();
        Imgproc.cvtColor(hsvImage, bgrImage, Imgproc.COLOR_HSV2BGR);

        // Enhance contrast using histogram equalization
        Mat ycrcbImage = new Mat();
        Imgproc.cvtColor(bgrImage, ycrcbImage, Imgproc.COLOR_BGR2YCrCb);

        List<Mat> ycrcbChannels = new ArrayList<>();
        Core.split(ycrcbImage, ycrcbChannels);

        Imgproc.equalizeHist(ycrcbChannels.get(0), ycrcbChannels.get(0));

        Core.merge(ycrcbChannels, ycrcbImage);

        Imgproc.cvtColor(ycrcbImage, bgrImage, Imgproc.COLOR_YCrCb2BGR);

        return bgrImage;
    }
    public static Mat process(Mat frame){
        Mat grayImage = new Mat();
        Imgproc.cvtColor(frame, grayImage, Imgproc.COLOR_BGR2GRAY);

        // Threshold the grayscale image to create a mask of the black pixels
        Mat mask = new Mat();
        Imgproc.threshold(grayImage, mask, 75, 255, Imgproc.THRESH_BINARY_INV); //75

        // Create a white image of the same size as the original image
        Mat whiteImage = new Mat(frame.size(), frame.type(), new Scalar(255, 255, 255));

        // Set the values of the white image to the original image where the mask is non-zero
        whiteImage.setTo(new Scalar(0, 0, 0), mask);
        return whiteImage;
    }
    public static Result readQRCode(Mat frame) {

        Bitmap bitmap = matToBitmap(frame);
        int[] pixels = new int[bitmap.getWidth() * bitmap.getHeight()];
        bitmap.getPixels(pixels, 0, bitmap.getWidth(), 0, 0, bitmap.getWidth(), bitmap.getHeight());
        RGBLuminanceSource source = new RGBLuminanceSource(bitmap.getWidth(), bitmap.getHeight(), pixels);
        BinaryBitmap binaryBitmap = new BinaryBitmap(new HybridBinarizer(source));
        MultiFormatReader reader = new MultiFormatReader();
        Result result;
        try {
            result = reader.decode(binaryBitmap);
            return result;
        } catch (NotFoundException e) {
            return null;
        }
    }

    public static Bitmap matToBitmap(Mat mat) {
        Bitmap bitmap = Bitmap.createBitmap(mat.cols(), mat.rows(), mat.channels() == 1 ? Bitmap.Config.ALPHA_8 : Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(mat, bitmap);
        return bitmap;
    }
}