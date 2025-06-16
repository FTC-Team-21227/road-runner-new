package org.firstinspires.ftc.teamcode.vision;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Core;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;

public class WatershedDetectionPipeline extends OpenCvPipeline {
    private Telemetry telemetry;
    private int objectCount = 0;
    private Scalar boundaryColor = new Scalar(0, 255, 0); // Green for boundaries
    private Scalar objectColor = new Scalar(255, 0, 0);    // Blue for object centers
    private boolean drawResults = true;

    public WatershedDetectionPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat result = input.clone();

        try {
            // Convert to grayscale
            Mat gray = new Mat();
            Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);

            // Apply blur
            Mat blurred = new Mat();
            Imgproc.GaussianBlur(gray, blurred, new Size(5, 5), 0);

            // Thresholding
            Mat thresh = new Mat();
            Imgproc.threshold(blurred, thresh, 0, 255, Imgproc.THRESH_BINARY_INV + Imgproc.THRESH_OTSU);

            // Noise removal
            Mat kernel = Mat.ones(3, 3, CvType.CV_8U);
            Mat opening = new Mat();
            Imgproc.morphologyEx(thresh, opening, Imgproc.MORPH_OPEN, kernel, new Point(-1, -1), 2);

            // Sure background area
            Mat sureBg = new Mat();
            Imgproc.dilate(opening, sureBg, kernel, new Point(-1, -1), 3);

            // Finding sure foreground area
            Mat distTransform = new Mat();
            Imgproc.distanceTransform(opening, distTransform, Imgproc.DIST_L2, 5);

            // Normalize and threshold
            Core.normalize(distTransform, distTransform, 0, 1.0, Core.NORM_MINMAX);
            Mat sureFg = new Mat();
            Imgproc.threshold(distTransform, sureFg, 0.7, 255, Imgproc.THRESH_BINARY);

            // Convert to 8-bit
            Mat sureFg8u = new Mat();
            sureFg.convertTo(sureFg8u, CvType.CV_8U);

            // Unknown region
            Mat unknown = new Mat();
            Core.subtract(sureBg, sureFg8u, unknown);

            // Marker labelling
            Mat markers = new Mat();
            Imgproc.connectedComponents(sureFg8u, markers);

            // Add one to all labels
            Core.add(markers, new Scalar(1), markers);

            // Mark the unknown region with 0
            markers.setTo(new Scalar(0), unknown);

            // Convert input to 3-channel for watershed
            Mat input3ch = new Mat();
            Imgproc.cvtColor(input, input3ch, Imgproc.COLOR_RGBA2RGB);

            // Perform watershed
            Imgproc.watershed(input3ch, markers);

            // Create boundary mask
            Mat boundaries = new Mat();
            Core.compare(markers, new Scalar(-1), boundaries, Core.CMP_EQ);

            if (drawResults) {
                // Draw boundaries on the result image
                result.setTo(boundaryColor, boundaries);

                // Find and draw object centers
                drawObjectCenters(result, markers);
            }

            // Count objects (regions with value > 1)
            Mat objects = new Mat();
            Core.compare(markers, new Scalar(1), objects, Core.CMP_GT);
            objectCount = Core.countNonZero(objects);

            telemetry.addData("Object Count", objectCount);
            telemetry.update();

            // Release Mats
            gray.release();
            blurred.release();
            thresh.release();
            opening.release();
            sureBg.release();
            distTransform.release();
            sureFg.release();
            sureFg8u.release();
            unknown.release();
            input3ch.release();
            boundaries.release();
            objects.release();

        } catch (Exception e) {
            telemetry.addData("Error", e.getMessage());
            telemetry.update();
        }

        return result;
    }

    private void drawObjectCenters(Mat image, Mat markers) {
        List<Point> centers = new ArrayList<>();
        List<Integer> markerValues = new ArrayList<>();

        // Find unique markers (excluding background and boundaries)
        for (int i = 0; i < markers.rows(); i++) {
            for (int j = 0; j < markers.cols(); j++) {
                double[] value = markers.get(i, j);
                if (value != null && value[0] > 1 && !markerValues.contains((int)value[0])) {
                    markerValues.add((int)value[0]);
                }
            }
        }

        // Find centers for each object
        for (int markerVal : markerValues) {
            Mat objectMask = new Mat();
            Core.compare(markers, new Scalar(markerVal), objectMask, Core.CMP_EQ);

            MatOfPoint contour = findLargestContour(objectMask);
            if (contour != null) {
                Moments moments = Imgproc.moments(contour);
                if (moments.m00 != 0) {
                    Point center = new Point(
                            moments.m10 / moments.m00,
                            moments.m01 / moments.m00
                    );
                    centers.add(center);

                    // Draw center point
                    Imgproc.circle(image, center, 5, objectColor, -1);

                    // Draw bounding rectangle
                    Rect rect = Imgproc.boundingRect(contour);
                    Imgproc.rectangle(image, rect.tl(), rect.br(), objectColor, 2);

                    // Draw marker value (for debugging)
                    Imgproc.putText(image, String.valueOf(markerVal),
                            new Point(rect.x, rect.y - 5),
                            Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, objectColor, 1);
                }
            }
            objectMask.release();
        }
    }

    private MatOfPoint findLargestContour(Mat mask) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        if (contours.isEmpty()) return null;

        // Find contour with maximum area
        double maxArea = 0;
        MatOfPoint largestContour = null;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                largestContour = contour;
            }
        }

        return largestContour;
    }

    public int getObjectCount() {
        return objectCount;
    }

    public void setDrawResults(boolean enabled) {
        this.drawResults = enabled;
    }

    public void setBoundaryColor(Scalar color) {
        this.boundaryColor = color;
    }

    public void setObjectColor(Scalar color) {
        this.objectColor = color;
    }
}