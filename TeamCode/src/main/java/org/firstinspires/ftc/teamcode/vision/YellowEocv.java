package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class YellowEocv extends OpenCvPipeline {

    public Mat watershed(Mat input){
        // Distance Transform
        Mat dist = new Mat();
        Imgproc.distanceTransform(binaryMat, dist, Imgproc.DIST_L2, 5);
        Core.normalize(dist, dist, 0, 1.0, Core.NORM_MINMAX);

// Threshold to get sure foreground
        Mat sureFg = new Mat();
        Imgproc.threshold(dist, sureFg, 0.4, 1.0, Imgproc.THRESH_BINARY);

// Convert to 8-bit image
        sureFg.convertTo(sureFg, CvType.CV_8U);

// Find contours on sure foreground
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(sureFg.clone(), contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

// Create markers image for watershed
        Mat markers = Mat.zeros(sureFg.size(), CvType.CV_32S);
        for (int i = 0; i < contours.size(); i++) {
            Imgproc.drawContours(markers, contours, i, new Scalar(i + 1), -1);
        }

// Add background marker (label 255)
        Imgproc.circle(markers, new Point(5,5), 3, new Scalar(255), -1);

// Apply watershed
        Mat inputRGB = new Mat();
        Imgproc.cvtColor(input, inputRGB, Imgproc.COLOR_RGBA2RGB);
        Imgproc.watershed(inputRGB, markers);

        Mat contourMat = input.clone();
// Draw results
        for (int i = 1; i <= contours.size(); i++) {
            Mat mask = new Mat();
            Core.compare(markers, new Scalar(i), mask, Core.CMP_EQ);

            // Find bounding rectangle
            List<MatOfPoint> singleContours = new ArrayList<>();
            Imgproc.findContours(mask, singleContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            for (MatOfPoint c : singleContours) {
                if (Imgproc.contourArea(c) > 100) {
                    Rect rect = Imgproc.boundingRect(c);
                    Imgproc.rectangle(input, rect.tl(), rect.br(), new Scalar(0, 255, 0), 2);
                    Imgproc.drawContours(contourMat, Collections.singletonList(c), -1, new Scalar(0, 255, 0), 1);
                }
            }
        }
        return contourMat;
    }
    /*
     * These are our variables that will be
     * modifiable from the variable tuner.
     *
     * Scalars in OpenCV are generally used to
     * represent color. So our values in the
     * lower and upper Scalars here represent
     * the Y, Cr and Cb values respectively.
     *
     * YCbCr, like most color spaces, range
     * from 0-255, so we default to those
     * min and max values here for now, meaning
     * that all pixels will be shown.
     */
    public Scalar lower = new Scalar(45.3, 151.6, 0);
    public Scalar upper = new Scalar(198.3, 191.3, 90.7);

    /**
     * This will allow us to choose the color
     * space we want to use on the live field
     * tuner instead of hardcoding it
     */
    public ColorSpace colorSpace = ColorSpace.YCrCb;

    /*
     * A good practice when typing EOCV pipelines is
     * declaring the Mats you will use here at the top
     * of your pipeline, to reuse the same buffers every
     * time. This removes the need to call mat.release()
     * with every Mat you create on the processFrame method,
     * and therefore, reducing the possibility of getting a
     * memory leak and causing the app to crash due to an
     * "Out of Memory" error.
     */
    private Mat ycrcbMat = new Mat();
    private Mat binaryMat = new Mat();
    private Mat maskedInputMat = new Mat();

    private Telemetry telemetry = null;

    /**
     * Enum to choose which color space to choose
     * with the live variable tuner isntead of
     * hardcoding it.
     */
    enum ColorSpace {
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
        ColorSpace(int cvtCode) {
            this.cvtCode = cvtCode;
        }
    }

    public YellowEocv(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        /*
         * Converts our input mat from RGB to
         * specified color space by the enum.
         * EOCV ALWAYS returns RGB mats, so you'd
         * always convert from RGB to the color
         * space you want to use.
         *
         * Takes our "input" mat as an input, and outputs
         * to a separate Mat buffer "ycrcbMat"
         */

        Imgproc.cvtColor(input, ycrcbMat, colorSpace.cvtCode);

        /*
         * This is where our thresholding actually happens.
         * Takes our "ycrcbMat" as input and outputs a "binary"
         * Mat to "binaryMat" of the same size as our input.
         * "Discards" all the pixels outside the bounds specified
         * by the scalars above (and modifiable with EOCV-Sim's
         * live variable tuner.)
         *
         * Binary meaning that we have either a 0 or 255 value
         * for every pixel.
         *
         * 0 represents our pixels that were outside the bounds
         * 255 represents our pixels that are inside the bounds
         */
        Core.inRange(ycrcbMat, lower, upper, binaryMat);

        /*
         * Release the reusable Mat so that old data doesn't
         * affect the next step in the current processing
         */
        maskedInputMat.release();

        /*
         * Now, with our binary Mat, we perform a "bitwise and"
         * to our input image, meaning that we will perform a mask
         * which will include the pixels from our input Mat which
         * are "255" in our binary Mat (meaning that they're inside
         * the range) and will discard any other pixel outside the
         * range (RGB 0, 0, 0. All discarded pixels will be black)
         */
        Core.bitwise_and(input, input, maskedInputMat, binaryMat);

        Mat contourMat = input.clone();

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(binaryMat, binaryMat, Imgproc.MORPH_OPEN, kernel);


        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(binaryMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            double epsilon = 0.02 * Imgproc.arcLength(contour2f, true); // the higher epsilon is, the more complex the polygon can be
            MatOfPoint2f approx = new MatOfPoint2f();
            Imgproc.approxPolyDP(contour2f, approx, epsilon, true);
            MatOfPoint polygon = new MatOfPoint(approx.toArray());

            // Draw the polygon in red
            Imgproc.drawContours(contourMat, Collections.singletonList(polygon), -1, new Scalar(0, 255, 0), 1);
        }


        // Add some nice and informative telemetry messages
        telemetry.addData("[Color Space]", colorSpace.name());
        telemetry.update();

        /*
         * The Mat returned from this method is the
         * one displayed on the viewport.
         *
         * To visualize our threshold, we'll return
         * the "masked input mat" which shows the
         * pixel from the input Mat that were inside
         * the threshold range.
         */

        return contourMat;
    }

}