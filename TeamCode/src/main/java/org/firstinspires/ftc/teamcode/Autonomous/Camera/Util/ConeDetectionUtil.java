package org.firstinspires.ftc.teamcode.Autonomous.Camera.Util;

import org.opencv.core.Mat;
import org.opencv.core.Point;

public interface ConeDetectionUtil {
    Mat extractColors(final Mat src);
    Mat processImage(final Mat mat);
    void markOuterContour(final Mat processedImage,
                          final Mat originalImage);
    void displayCentre(Mat originalImage);
    RelativePosition getRelativePosition (Point center, Mat image);

}
