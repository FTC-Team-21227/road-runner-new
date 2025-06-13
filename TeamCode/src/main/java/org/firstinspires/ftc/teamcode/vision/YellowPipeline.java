package org.firstinspires.ftc.teamcode.vision;

//import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;


//@Config
public class YellowPipeline extends OpenCvPipeline {
    public static Scalar lowerYellow = new Scalar(130, 130, 0);
    public static Scalar upperYellow = new Scalar(255, 175, 110);
    public static boolean pipelineOn = true;
    /**
     * This will allow us to choose the color
     * space we want to use on the live field
     * tuner instead of hardcoding it
     */
    ColorSpace colorSpace = ColorSpace.YCrCb;

    /*
     * declaring the Mats you will use here at the top
     * of your pipeline to prevent "out of memory" error
     */
    private Mat ycrcbMat       = new Mat();
    private Mat binaryMat      = new Mat();
    private Mat maskedInputMat = new Mat();
    private Mat blurredMat = new Mat();

    private Telemetry telemetry = null;

    /**
     * Enum to choose which color space to choose
     * with the live variable tuner instead of
     * hardcoding it.
     */
    enum ColorSpace
    {
        /*
         * Define our "conversion codes" in the enum
         * so that we don't have to do a switch
         * statement in the processFrame method.
         */
        RGB(Imgproc.COLOR_RGBA2RGB),
        HSV(Imgproc.COLOR_RGB2HSV),
        YCrCb(Imgproc.COLOR_RGB2YCrCb),
        Lab(Imgproc.COLOR_RGB2Lab);

        //store cvtCode in a public var
        public int cvtCode = 0;

        //constructor to be used by enum declarations above
        ColorSpace(int cvtCode)
        {
            this.cvtCode = cvtCode;
        }
    }

    public YellowPipeline(Telemetry telemetry)
    {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input)
    {
        Imgproc.cvtColor(input, ycrcbMat, colorSpace.cvtCode);
        Core.inRange(ycrcbMat, lowerYellow, upperYellow, binaryMat);

        Core.bitwise_and(input, input, maskedInputMat, binaryMat);

        // Find contours
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        Mat grayMaksedInput = new Mat();

        Imgproc.cvtColor(maskedInputMat, grayMaksedInput, Imgproc.COLOR_YCrCb2RGB);
        Imgproc.cvtColor(maskedInputMat, grayMaksedInput, Imgproc.COLOR_RGB2GRAY);
        Imgproc.findContours(maskedInputMat, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        // Store contours
        ArrayList<ArrayList<Point3>> contour3DList = new ArrayList<>();

        for (MatOfPoint contour : contours) {
            // Approximate polygonal curve
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            double epsilon = 0.02 * Imgproc.arcLength(contour2f, true);
            MatOfPoint2f approx = new MatOfPoint2f();
            Imgproc.approxPolyDP(contour2f, approx, epsilon, true);

            // Convert to 3D points (Z=0)
            ArrayList<Point3> contour3D = new ArrayList<>();
            for (Point pt : approx.toArray()) {
                contour3D.add(new Point3(pt.x, pt.y, 0));
            }
            contour3DList.add(contour3D);

            // Draw polygon on original image
            MatOfPoint polygon = new MatOfPoint(approx.toArray());
            Imgproc.drawContours(maskedInputMat, Collections.singletonList(polygon), -1, new Scalar(0, 0, 255), 2);
        }

        // Print 3D points for use in 3D viewer
        for (int i = 0; i < contour3DList.size(); i++) {
            telemetry.addData("Contour", i);
            for (Point3 p : contour3DList.get(i)) {
                telemetry.addData("x-coor", p.x);
                telemetry.addData("y-coor", p.y);
                telemetry.addData("z-coor", p.z);
            }
        }

        /**
         * Add some nice and informative telemetry messages
         */
        telemetry.addData("[>]", "Change these values in tuner menu");
        telemetry.addData("[Color Space]", colorSpace.name());
        telemetry.addData("[Lower Scalar]", lowerYellow);
        telemetry.addData("[Upper Scalar]", upperYellow);
        telemetry.update();
        if (pipelineOn) {
            return maskedInputMat;
        }
        else{
            return input;
        }
    }
}