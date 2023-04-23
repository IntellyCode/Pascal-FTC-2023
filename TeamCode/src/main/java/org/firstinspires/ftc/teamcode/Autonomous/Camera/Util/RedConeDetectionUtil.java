package org.firstinspires.ftc.teamcode.Autonomous.Camera.Util;

import org.firstinspires.ftc.teamcode.Autonomous.Camera.Pipeline;
import org.opencv.core.Point;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

//import android.graphics.Point;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;


public class RedConeDetectionUtil implements ConeDetectionUtil {

    public RedConeDetectionUtil() {

    }
    public Mat extractColors(final Mat src) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(src, hsv, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(0, 60, 100);
        Scalar highHSV = new Scalar(15, 255, 255);



        Mat blueMask = new Mat();
        Core.inRange(hsv, lowHSV, highHSV, blueMask);

        // Apply a morphological closing operation to fill small gaps in the detected contour
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(blueMask, blueMask, Imgproc.MORPH_CLOSE, kernel);

        Mat result = new Mat();
        src.copyTo(result, blueMask);

        return result;
    }

    public Mat processImage(final Mat mat) {
        final Mat processed = new Mat(mat.height(), mat.width(), mat.type());
        // Increase the Gaussian blur kernel size to reduce noise and smoothen the edges
        Imgproc.GaussianBlur(mat, processed, new Size(9, 9), 1);

        Imgproc.cvtColor(processed, processed, Imgproc.COLOR_RGB2GRAY);

        Imgproc.Canny(processed, processed, 200, 25);

        Imgproc.dilate(processed, processed, new Mat(), new Point(-1, -1), 2);

        return processed;
    }


   public void markOuterContour(final Mat processedImage,
                                       final Mat originalImage) {
       // Find contours of an image

       final List<MatOfPoint> allContours = new ArrayList<>();
       Imgproc.findContours(
               processedImage,
               allContours,
               new Mat(processedImage.size(), processedImage.type()),
               Imgproc.RETR_EXTERNAL,
               Imgproc.CHAIN_APPROX_NONE
       );
       // Filter out noise and display contour
       final List<MatOfPoint> filteredContours = allContours.stream()
               .filter(contour -> {
                   final double value = Imgproc.contourArea(contour);

                   final boolean isNotNoise = value > 2000;

                   return isNotNoise;
               }).collect(Collectors.toList());

       if (filteredContours.isEmpty()) {
//       Pipeline.currentAction=State.toPole;
           Pipeline.coneSelected = false;
       } else {
//          Pipeline.currentAction=State.toCone;
           Pipeline.coneSelected = true;
       }
           // draw Contours
           for (MatOfPoint contour : filteredContours) {
               final Rect rect = Imgproc.boundingRect(contour);
               final Point center = new Point(rect.x + rect.width / 2, rect.y + rect.height / 2);
               Imgproc.drawContours(
                       originalImage,
                       Collections.singletonList(contour),
                       -1, // Negative value indicates that we want to draw all of contours
                       new Scalar(255, 0, 0),
                       4
               );

               // Draw green dot at the center of the contour
               Imgproc.circle(
                       originalImage,
                       center,
                       2,
                       new Scalar(0, 255, 0),
                       -1
               );

               // Display coordinates of the center point in green text
               Imgproc.putText(
                       originalImage,
                       "Center: (" + (int) center.x + ", " + (int) center.y + ")" + Imgproc.contourArea(contour),
                       new Point(center.x - 60, center.y - 10),
                       2,
                       0.5,
                       new Scalar(0, 255, 0),
                       1
               );
               Pipeline.conePosition = getRelativePosition(center, originalImage);
               Pipeline.coneArea = Imgproc.contourArea(contour);
           }

           displayCentre(originalImage);


       }

       public void displayCentre (Mat originalImage){
           // Calculate the center of the input image
           int centerX = originalImage.cols() / 2;
           int centerY = originalImage.rows() / 2;
           Point centerOfImage = new Point(centerX, centerY);

           // Draw an orange point at the center of the input image
           Scalar orangeColor = new Scalar(255, 165, 0);
           int pointRadius = 2;
           int pointThickness = -1; // Filled circle
           Imgproc.circle(originalImage, centerOfImage, pointRadius, orangeColor, pointThickness);

           // Display coordinates of the center of the input image
           String centerCoordinatesText = "Center: (" + centerX + ", " + centerY + ")";
           Point textPosition = new Point(centerX - 60, centerY - 10);
           int fontFace = 2;
           double fontScale = 0.5;
           int textThickness = 1;
           Imgproc.putText(originalImage, centerCoordinatesText, textPosition, fontFace, fontScale, orangeColor, textThickness);

       }

       public RelativePosition getRelativePosition (Point center, Mat image){

           Point imageCenter = new Point(image.cols() / 2, image.rows() / 2);

           if (imageCenter.x + 80 < center.x) {
               return RelativePosition.right;
           } else if (imageCenter.x - 80 > center.x) {
               return RelativePosition.left;
           } else {
               return RelativePosition.center;
           }
    }
}