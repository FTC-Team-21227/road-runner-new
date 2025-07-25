package org.firstinspires.ftc.teamcode.vision;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.min;
import static java.lang.Math.pow;
import static java.lang.Math.round;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystem_Constants;
import org.firstinspires.ftc.teamcode.TunePID_MotionProfile;
import org.firstinspires.ftc.teamcode.autons.PoseStorage;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

@Config
public class ExcludePipeline extends OpenCvPipeline {
    public static int retVal = 0; // 0: original image, 1: binary image, 2: edges
    public static boolean isBlue = true;
    List<MatOfPoint> contours = new ArrayList<>();
    boolean chamberPos;

    public static boolean printStuff= false; // Telemetry and outputting intermediate mats

    // THRESHOLDS
    // HSV thresholds
    public static double RUH = 10, RLH = 130, RS = 90, RV = 180; // upper and lower H, S, and V
    public static double BH = 110, BUH = 130, BS = 70, BV = 190;
    public static double YH = 15, YUH = 40, YS = 80, YV = 210;
    public static double AREA_RATIO_WEIGHT = -0.4, UPPIES = .5, MIN_AREA = 2500 /*7000*/;
    /* Canny edge detection thresholds
     * Strong edges when intensity > UPPER_THRESH (definite edges)
     * Weak edges when LOWER_THRESH < intensity < UPPER_THRESH (edge candidates)
     * Weak edges included when connected to strong edge; all others discarded
     */
    public static int UPPER_THRESH = 280 /*120*/, LOWER_THRESH = 20 /*60*/, YUPPER_THRESH = 240, YLOWER_THRESH = 80;
    public static int KERNEL_SIZE = 2, YELLOW_KERNEL_SIZE = 2;

    // Calibration and conversions
    public static double horizontal_offset, camera_tilt, forward_offset, inchPerPixel_x, inchPerPixel_y, k, MIN_DIST = 36;

    // Intermediate OpenCV Mats
    Mat hsv = new Mat();
    Mat mask = new Mat(), mask2 = new Mat(), closedEdges = new Mat(), edges = new Mat();
    Mat kernel = new Mat();
    Mat colorMask = new Mat(), colorMask2 = new Mat();

    Mat hierarchy = new Mat();
    Mat boundingImage = new Mat(), maskedImage = new Mat();

    // Aspect ratio, matching thresholds
    public static double AREA_THRESH = .6 /*.82*/, FCL = 1, UP_TOLERANCE = 0.6, DOWN_TOLERANCE = 0.8, CLASSUP_TOL = 0.5, CLASSDOWN_TOL = 0.3;

    // IRL Sample measurements
    double objectWidth = 3.5;
    double objectHeight = 1.5;

    // Used for process bounding rectangles
    MatOfPoint2f contour2f = new MatOfPoint2f();
    private volatile double[] center = {0, 0, 0, 0};
    Double[] camCent = {0.0 , 0.0, 0.0, 0.0};

    // Object position in camera stream
    double objX_cam = 0, objY_cam = 0, objZ_cam = 0;
    double objX_base = 0, objY_base = 0;
    int color = 0; // Color mode: 0 is red, 1 is blue, 2 is red/blue + yellow depending on isBlue bool

    Mat cameraMatrix = new Mat(3, 3, CvType.CV_64FC1);
    MatOfDouble distCoeffs = new MatOfDouble();
    RotatedRect minAreaRect;
    Telemetry telemetry;


    // CAMERA CALIBRATION
    public ExcludePipeline(Telemetry telemetry, boolean chamberPos) {
        if (chamberPos) {
                horizontal_offset = -9.5 /*5*/ /*-11.5*/;
                camera_tilt = Math.toRadians(36);
                forward_offset = 0;
                inchPerPixel_x = 17.0/640;
                inchPerPixel_y = 18.0/480;
                k = Math.log(14.0/2)/Math.log(14.17/3.68);} //30; //45; //50 /*30*/;}
        else {
            horizontal_offset = 7 /*-8*/ /*5*/ /*-9.5*/;
            camera_tilt = Math.toRadians(0);
            forward_offset = -6.5 /*-2.5*/ /*-3*/;
            inchPerPixel_x = 10.5/640;
            inchPerPixel_y = 7.0/480;
            k=1;
        } //30; //45; //50 /*30*/;}
//        double fx = 1647 * FCL; // Replace with your camera's focal length in pixels
//        double fy = 1647 * FCL;
//        double cx = 746; // Replace with your camera's principal point x-coordinate (usually image width / 2)
//        double cy = 439; // Replace with your camera's principal point y-coordinate (usually image height / 2)
//        cameraMatrix.put(0, 0,
//                fx, 0, cx,
//                0, fy, cy,
//                0, 0, 1);
//        distCoeffs = new MatOfDouble(0.08642896,  0.58342025,  0.00830023,  0.00885814, -3.45247042);
        this.telemetry = telemetry;
        this.chamberPos = chamberPos;
    }

    // Reset detected object's center to 0
    public void resetCenter() {
        center = new double[]{0, 0, 0, 0};
    }

    public ExcludePipeline(Telemetry telemetry)
    {
        this.telemetry = telemetry;
    }

    // FRAME PROCESSING
    @Override
    public Mat processFrame(Mat input) {
        // Convert input image to HSV colors for better color detection
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // FILTERS FOR COLORS (SCALARS)
        Scalar rlFilt = new Scalar(RLH, RS, RV),    // red lower filter
                ruFilt = new Scalar(180, 255, 255), // red upper filter
                rllFilt = new Scalar(0, RS, RV),
                rulFilt = new Scalar(RUH, 255, 255),
                blFilt = new Scalar(BH, BS, BV),
                buFilt = new Scalar(BUH, 255, 255),
                ylFilt = new Scalar(YH, YS, YV),
                yuFilt = new Scalar(YUH, 255, 255);

        input.copyTo(boundingImage);

        // CREATING MASKS BASED ON COLOR
        if (color == 0) { // Red
            Core.inRange(hsv, rlFilt, ruFilt, mask);
            Core.inRange(hsv, rllFilt, rulFilt, mask2);
            Core.bitwise_or(mask, mask2, colorMask); // Combines both masks
        }
        else if (color == 1) { // Blue
            Core.inRange(hsv, blFilt, buFilt, colorMask);
        }
        else if(color == 2) { // Red/Blue + Yellow
            if (isBlue) { // Blue alliance
                Core.inRange(hsv, blFilt, buFilt, colorMask);
            }
            else { // Red alliance
                Core.inRange(hsv, rlFilt, ruFilt, mask);
                Core.inRange(hsv, rllFilt, rulFilt, mask2);
                Core.bitwise_or(mask, mask2, colorMask);
            }
            Core.inRange(hsv, ylFilt, yuFilt, colorMask2); // Adding yellows
        }
        else { // Just yellow
            Core.inRange(hsv, ylFilt, yuFilt, colorMask);
        }


        // INPUT MAT WITH A MASK
        maskedImage = new Mat();
        Core.bitwise_and(input, input, maskedImage, colorMask); // Store masked image into maskedImage mat
        if (color == 2) { // Red/Blue + Yellow
            Core.bitwise_and(input, input, maskedImage, colorMask2);
        }

        edges = new Mat();

        // EDGE DETECTION
        if (color != 2) { // Red/Blue
            Imgproc.Canny(maskedImage, edges, LOWER_THRESH, UPPER_THRESH);                                      // Canny edge detection (edge = black, non-edge = white)
            kernel = Imgproc.getStructuringElement(Imgproc.MORPH_DILATE, new Size(KERNEL_SIZE, KERNEL_SIZE));   // Kernel: Dilation to expand white areas
            closedEdges = new Mat();                                                                            // Mat to store dilated result with closed edges
            Imgproc.dilate(edges, closedEdges, kernel);                                                         // Dilate the mat using kernel
            kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(KERNEL_SIZE, KERNEL_SIZE));     // Kernel: Rectangular closing edges
            Imgproc.morphologyEx(closedEdges, edges, Imgproc.MORPH_CLOSE, kernel);                              // Close the edges using kernel
        }
        else { // Red/Blue + Yellow
            Imgproc.Canny(maskedImage, edges, YLOWER_THRESH, YUPPER_THRESH);
            kernel = Imgproc.getStructuringElement(Imgproc.MORPH_DILATE, new Size(YELLOW_KERNEL_SIZE, YELLOW_KERNEL_SIZE));
            closedEdges = new Mat();
            Imgproc.dilate(edges, closedEdges, kernel);
            kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(YELLOW_KERNEL_SIZE, YELLOW_KERNEL_SIZE));
            Imgproc.morphologyEx(closedEdges, edges, Imgproc.MORPH_CLOSE, kernel);
        }

        // FIND CONTOURS
        contours = new ArrayList<>(); // List to store contours; each contour is a MatOfPoint representing boundary of a closed shape
        Imgproc.findContours(closedEdges, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE); // (binary image, contour list, parent/child nested, get all contours/no nesting, simplify lines)
        ArrayList<Double[]> colorCoords = contoursToCoords(); // ArrayList of all center coordinates

        // FINDING BEST CENTER'S COORDINATES AND TELEMETRY LINES
        if (!contours.isEmpty()) {
            Double[] centerd = matchedCoords(colorCoords, colorCoords);                // Coordinates of center of best contour
            if (centerd[0] != 100) center = convertToDoubleArray(centerd);             // If center is found, convert Double[] to double[]
            if (printStuff) {
            telemetry.addData("Valid Center", center[0] != 0);          // If center is valid
            telemetry.addData("Contour Count", contours.size());              // Count of contours found
            telemetry.addData("ColorCoords Count", colorCoords.size());}      // Count of ColorCoords
        }
        else if (printStuff){
            telemetry.addLine("Contours are empty!!");
        }

        if (printStuff) telemetry.update();


        // MAT OUTPUT:
        Mat result;
        if(printStuff) {
            if (retVal == 0) {result = boundingImage;}
            else if (retVal == 1) {result = maskedImage;}
            else if (retVal == 2) {result = edges;}
            else {result = closedEdges;}
        }
        else {result = input;}

        // RELEASE UNUSED MATS
        if (result != closedEdges) closedEdges.release();
        if (result != edges) edges.release();
        if (result != maskedImage) maskedImage.release();
        if (result != hsv) hsv.release();
        if (result != mask) mask.release();
        if (result != mask2) mask2.release();
        if (result != hierarchy) hierarchy.release();
        if (result != colorMask) colorMask.release();
        if (result != colorMask2) colorMask2.release();
        if (result != boundingImage) boundingImage.release();

        return result;
    } // END OF FRAME PROCESSING


    // Double[] to double[] (used to store center coordinates)
    double[] convertToDoubleArray(Double[] wrapperArray) {
        double[] primitiveArray = new double[wrapperArray.length];

        for (int i = 0; i < wrapperArray.length; i++) {
            primitiveArray[i] = wrapperArray[i]; // Auto-unboxing
        }
        return primitiveArray;
    }


    // Setter for center
    public synchronized void setCenter(double[] newCenter) {
        center = newCenter;
    }


    // Getter for center (adds values to dashboard packet if printStuff enabled)
    public synchronized double[] getCenter(@NonNull TelemetryPacket packet) {
        if (printStuff) {
        packet.put("objX_cam", objX_cam);
        packet.put("objY_cam", objY_cam);
        packet.put("objZ_cam", objZ_cam);
        packet.put("CAM X", camCent[0]);
        packet.put("CAM Y", camCent[1]);
        packet.put("CAM Z", camCent[2]);
        packet.put("angle", camCent[3]);
        packet.put("objX_base", objX_base);
        packet.put("objY_base", objY_base);
        }
        return center;
    }


    // Camera-relative to robot-relative object position, finding closest object. Returns center coordinates (field)
    public Double[] matchedCoords(ArrayList<Double[]> colorCoords, ArrayList<Double[]> allCoords) {
        ArrayList<Double[]> matchedCenters = new ArrayList<>();
        double minDist = MIN_DIST; // Start with maximum allowed distance
        int coord = 0;             // Index of center coordinate

        // Loop through each detected object center
        for (int i = 0; i < colorCoords.size(); i++) {
            Double[] relCent = colorCoords.get(i).clone(); // Current object's camera-relative coordinates
            // Camera-relative coordinates
            objX_cam = relCent[0];  // Lateral
            objY_cam = relCent[1];  // Vertical
            objZ_cam = relCent[2];  // Depth (forward)
            double angle = relCent[3]; // Orientation/rotation of object

            // Field-coordinate X
            if (chamberPos) {
                if (PoseStorage.grabColorPose.position.x < 5) horizontal_offset += 6; // Offset depending on current pose
                objX_base = objX_cam + horizontal_offset; // Distance of object from base of robot
            }
            else
                objX_base = -objX_cam + horizontal_offset; // Invert

            // Field-coordinate Y
            if (chamberPos)
                objY_base = 18.0 * Math.pow((-objY_cam + 18.0) / 18.0, k) + forward_offset;
            else
                objY_base = 7.0 * Math.pow((objY_cam) / 7.0, k) + forward_offset;

            // Calculate distance to end effector
            double endEffectorX = 0; // Fixed x-position
            double endEffectorY;

            if (chamberPos) {endEffectorY = PoseStorage.grabColorPose.position.y + 45;}
            else {endEffectorY = PoseStorage.grabYellowPose.position.y - 94;}

            // Squared distance to end effector
            double dx = objX_base - endEffectorX;
            double dy = objY_base - endEffectorY;
            double dist = dx*dx + dy*dy;

            if (dist < minDist && angle > 50 && angle < 120) {
                coord = i; // Save the index
                minDist = dist; // Update best distance
                // Store transformed coordinates for later use
                relCent[0] = objX_base; // Overwrite with field coordinates
                relCent[1] = objY_base;
                matchedCenters.add(relCent); // Save matched center
            }
        }

        if (matchedCenters.isEmpty()){ // No good centers found
            return new Double[]{100.0, 100.0, 100.0, 100.0};} // Sentinel: "not found"
        else {
            return matchedCenters.get(matchedCenters.size() - 1); // Return closest (latest) match
        }
    }


    // CREATE RECTANGLE AROUND BEST CONTOUR, RETURNS CENTER COORDINATES
    public ArrayList<Double[]> contoursToCoords() {
        ArrayList<Double[]> centers = new ArrayList<>();

        // Set acceptable aspect ratio range with tolerance
        double minAspectRatio = 3.5 / 1.5 - DOWN_TOLERANCE;
        double maxAspectRatio = 3.5 / 1.5 + UP_TOLERANCE;

        // Iterate over contours
        for (MatOfPoint contour : contours) {
            // Skip small contours
            if (Imgproc.contourArea(contour) < MIN_AREA) {continue;}

            // Convert contour points to MatOfPoint2f, get area of the contour
            contour2f = new MatOfPoint2f(contour.toArray());
            minAreaRect = Imgproc.minAreaRect(contour2f); // Minimum area of contour rectangle

            if (minAreaRect.size.width != 0 && minAreaRect.size.height != 0 &&
                    Imgproc.contourArea(contour) / (minAreaRect.size.height * minAreaRect.size.width) > AREA_THRESH) {
                Point[] box = new Point[4];
                minAreaRect.points(box); // Get 4 points of bounding rectangle and store in box
                Point[] orded = orderPoints(box); // Order the points

                // DELETE THIS BOUNDIGN BOX CODE? REPEATED
//                if(printStuff) { // Draw red bounding box
//                    for (int j = 0; j < 4; j++) {
//                        Imgproc.line(boundingImage, box[j], box[(j + 1) % 4], new Scalar(255, 0, 0), 2);
//                    }
//                }

                // Calculate distances between corner points to get width and height
                double[] distances = {distance(orded[0], orded[1]), distance(orded[1], orded[2]), distance(orded[0], orded[2])};
                Arrays.sort(distances);
                double width = distances[1];
                double height = distances[0];

                if (height != 0) {  // Avoid division by zero
                    double aspectRatio = width / height;
//                  if (minAspectRatio <= aspectRatio && aspectRatio <= maxAspectRatio) {

                    // Get angle of rotation of rectangle
                    double rotRectAngle = minAreaRect.angle;
                    if (minAreaRect.size.width < minAreaRect.size.height) {
                        rotRectAngle += 90;
                    }

                    // Compute the angle and store it
                    double angle = (rotRectAngle);

                    // Draw blue bounding box
                    if(printStuff) {
                        for (int j = 0; j < 4; j++) {
                            Imgproc.line(boundingImage, box[j], box[(j + 1) % 4], new Scalar(0, 0, 255), 2);
                        }
                    }

                    // Calculate centroid of rectangle, convert to inches
                    double[] coords = new double[3];
                    coords[0] = (orded[0].x + orded[1].x + orded[2].x + orded[3].x) / 4.0 * inchPerPixel_x;
                    coords[1] = (orded[0].y + orded[1].y + orded[2].y + orded[3].y) / 4.0 * inchPerPixel_y;
                    coords[2] = (1000);

                    // Display on screen
                    if(printStuff) {
                        String label = "(" + round(coords[0]*100) + ", " + round(coords[1]*100) + ")";
                        Imgproc.putText(boundingImage, label, new Point(coords[0] + 10, coords[1] + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                    }

                    // STORE CAMERA-RELATIVE COORDINATES + ANGLE IN AN ARRAY; we'll transform in matchedCoords
                    camCent = new Double[]{
                            coords[0],  // Raw camera-relative X
                            coords[1],  // Raw camera-relative Y
                            coords[2],  // Raw camera-relative Z
                            angle};       // Detection angle
                    // Add the data to centers list
                    centers.add(camCent);
                }
            }
        }
        // Return list of detected centers and angles
        return centers;
    }


    private static double distance(Point p1, Point p2) {
        return Math.sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
    }

    // ORDERING POINTS
    public static Point[] orderPoints(Point[] pts) {
        if (pts.length != 4) {
            throw new IllegalArgumentException("Exactly four points are required.");
        }

        // Calculate the center of the frame
        double centerX = 320;
        double centerY = 240;
        Point center = new Point((int) centerX, (int) centerY);

        // Calculate distances from each point to the center
        double[] distances = new double[4];
        for (int i = 0; i < 4; i++) {
            distances[i] = distance(center, pts[i]);
        }

        // Sort points by proximity to center
        Point[] sortedByDistance = Arrays.copyOf(pts, 4);
        Arrays.sort(sortedByDistance, Comparator.comparingDouble(p -> distance(center, p)));

        // Start with the two closest points
        Point[] orderedPts = new Point[4];
        orderedPts[0] = sortedByDistance[0]; // Closest point
        orderedPts[1] = sortedByDistance[1]; // Second closest point

        // Remaining points
        Point thirdPoint = sortedByDistance[2];
        Point fourthPoint = sortedByDistance[3];

        // Determine the order of the remaining points in clockwise manner
        if (isCC(thirdPoint, orderedPts[0], orderedPts[1], fourthPoint)) {
            orderedPts = new Point[]{thirdPoint, orderedPts[0], orderedPts[1], fourthPoint};
        } else if (isCC(thirdPoint, orderedPts[1], orderedPts[0], fourthPoint)) {
            orderedPts = new Point[]{thirdPoint, orderedPts[1], orderedPts[0], fourthPoint};

        } else if (isCC(fourthPoint, orderedPts[1], orderedPts[0], thirdPoint)) {
            orderedPts = new Point[]{fourthPoint, orderedPts[1], orderedPts[0], thirdPoint};

        } else {
            orderedPts = new Point[]{fourthPoint, orderedPts[0], orderedPts[1], thirdPoint};

        }

        return orderedPts;
    }

    public static boolean isCC(Point p1, Point p2, Point p3, Point p4) {
        return isCCW(p1, p2, p3) && isCCW(p1, p3, p4) &&
                isCCW(p2, p3, p4) && isCCW(p1, p2, p4);
    }

    private static boolean isCCW(Point a, Point b, Point c) {
        // Calculate the cross product of vector AB and AC
        double crossProduct = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
        return crossProduct > 0; // Returns true if the points are in counter-clockwise order
    }

    // DISPLAYING INFO ON IMAGE
    static void drawTagText(RotatedRect rect, String text, Mat mat, String color) {
        Scalar colorScalar = getColorScalar(color); // Font color

        Imgproc.putText(
                mat, // The buffer we're drawing on
                text, // The text we're drawing
                new Point( // The anchor point for the text
                        rect.center.x - 50,  // x anchor point
                        rect.center.y + 25), // y anchor point
                Imgproc.FONT_HERSHEY_PLAIN, // Font
                1,                          // Font size
                colorScalar,                // Font color
                1);                         // Font thickness
    }

    // Font color for drawTagText()
    static Scalar getColorScalar(String color) {
        switch (color) {
            case "Blue":
                return new Scalar(0, 0, 255);
            case "Yellow":
                return new Scalar(255, 255, 0);
            default:
                return new Scalar(255, 0, 0);
        }
    }

    public void setColor(int color) {
        this.color = color;
    }

    public int getColor() {
        return color;
    }
}