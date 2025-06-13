package org.firstinspires.ftc.teamcode.autons;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystem_Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.List;
import java.util.Locale;

public class Camera {
    /*
     * EDIT THESE PARAMETERS AS NEEDED
     */
    final boolean USING_WEBCAM = false;
    final BuiltinCameraDirection INTERNAL_CAM_DIR = BuiltinCameraDirection.BACK;
    final int RESOLUTION_WIDTH = 640;
    final int RESOLUTION_HEIGHT = 480;

    // Internal state
    boolean lastX;
    int frameCount;
    long capReqTime;
    VisionPortal portal;
    ColorBlobLocatorProcessor colorLocator;

    public Camera(HardwareMap hardwareMap, boolean chamberPos, String color1, String color2) {
        /* Build a "Color Locator" vision processor based on the ColorBlobLocatorProcessor class.
         * - Specify the color range you are looking for.  You can use a predefined color, or create you own color range
         *     .setTargetColorRange(ColorRange.BLUE)                      // use a predefined color match
         *       Available predefined colors are: RED, BLUE YELLOW GREEN
         *     .setTargetColorRange(new ColorRange(ColorSpace.YCrCb,      // or define your own color match
         *                                           new Scalar( 32, 176,  0),
         *                                           new Scalar(255, 255, 132)))
         *
         * - Focus the color locator by defining a RegionOfInterest (ROI) which you want to search.
         *     This can be the entire frame, or a sub-region defined using:
         *     1) standard image coordinates or 2) a normalized +/- 1.0 coordinate system.
         *     Use one form of the ImageRegion class to define the ROI.
         *         ImageRegion.entireFrame()
         *         ImageRegion.asImageCoordinates(50, 50,  150, 150)  100x100 pixel square near the upper left corner
         *         ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5)  50% width/height square centered on screen
         *
         * - Define which contours are included.
         *     You can get ALL the contours, or you can skip any contours that are completely inside another contour.
         *        .setContourMode(ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY)  // return all contours
         *        .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)            // exclude contours inside other contours
         *        note: EXTERNAL_ONLY helps to avoid bright reflection spots from breaking up areas of solid color.
         *
         * - turn the display of contours ON or OFF.  Turning this on helps debugging but takes up valuable CPU time.
         *        .setDrawContours(true)
         *
         * - include any pre-processing of the image or mask before looking for Blobs.
         *     There are some extra processing you can include to improve the formation of blobs.  Using these features requires
         *     an understanding of how they may effect the final blobs.  The "pixels" argument sets the NxN kernel size.
         *        .setBlurSize(int pixels)    Blurring an image helps to provide a smooth color transition between objects, and smoother contours.
         *                                    The higher the number of pixels, the more blurred the image becomes.
         *                                    Note:  Even "pixels" values will be incremented to satisfy the "odd number" requirement.
         *                                    Blurring too much may hide smaller features.  A "pixels" size of 5 is good for a 320x240 image.
         *        .setErodeSize(int pixels)   Erosion removes floating pixels and thin lines so that only substantive objects remain.
         *                                    Erosion can grow holes inside regions, and also shrink objects.
         *                                    "pixels" in the range of 2-4 are suitable for low res images.
         *        .setDilateSize(int pixels)  Dilation makes objects more visible by filling in small holes, making lines appear thicker,
         *                                    and making filled shapes appear larger. Dilation is useful for joining broken parts of an
         *                                    object, such as when removing noise from an image.
         *                                    "pixels" in the range of 2-4 are suitable for low res images.
         */
         colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();

        /*
         * Build a vision portal to run the Color Locator process.
         *
         *  - Add the colorLocator process created above.
         *  - Set the desired video resolution.
         *      Since a high resolution will not improve this process, choose a lower resolution that is
         *      supported by your camera.  This will improve overall performance and reduce latency.
         *  - Choose your video source.  This may be
         *      .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))  .....   for a webcam
         *  or
         *      .setCamera(BuiltinCameraDirection.BACK)    ... for a Phone Camera
         */
        portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();
    }

    public class captureFrame implements Action {
        ElapsedTime time = new ElapsedTime();
        boolean start;
        double runTime;
        public captureFrame(double waittim){
            start = false;
            runTime = waittim;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!start) {
                time.reset();
                start = true;
            }
            if (time.seconds() < runTime) {
                return true;
            } else {
                portal.saveNextFrameRaw(String.format(Locale.US, "CameraFrameCapture-%06d", frameCount++));
                return false;
            }
        }
    }
    public Action snap(double tim) {
        return new captureFrame(tim);}

    public class computeTargetPose implements Action {
        ElapsedTime time = new ElapsedTime();
        boolean start;
        double runTime;
        public computeTargetPose(double waittim){
            start = false;
            runTime = waittim;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!start) {
                time.reset();
                start = true;
            }
            if (time.seconds() < runTime) {
                return true;
            }
            else {
                snap(0);
                // Read the current list
                List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
                /*
                 * The list of Blobs can be filtered to remove unwanted Blobs.
                 *   Note:  All contours will be still displayed on the Stream Preview, but only those that satisfy the filter
                 *          conditions will remain in the current list of "blobs".  Multiple filters may be used.
                 *
                 * Use any of the following filters.
                 *
                 * ColorBlobLocatorProcessor.Util.filterByArea(minArea, maxArea, blobs);
                 *   A Blob's area is the number of pixels contained within the Contour.  Filter out any that are too big or small.
                 *   Start with a large range and then refine the range based on the likely size of the desired object in the viewfinder.
                 *
                 * ColorBlobLocatorProcessor.Util.filterByDensity(minDensity, maxDensity, blobs);
                 *   A blob's density is an indication of how "full" the contour is.
                 *   If you put a rubber band around the contour you would get the "Convex Hull" of the contour.
                 *   The density is the ratio of Contour-area to Convex Hull-area.
                 *
                 * ColorBlobLocatorProcessor.Util.filterByAspectRatio(minAspect, maxAspect, blobs);
                 *   A blob's Aspect ratio is the ratio of boxFit long side to short side.
                 *   A perfect Square has an aspect ratio of 1.  All others are > 1
                 */
                ColorBlobLocatorProcessor.Util.filterByArea(50,20000,blobs);  // filter out very small blobs.

                if (blobs.isEmpty() && time.seconds() < runTime + 1){
                    return true;
                }
                /*
                 * The list of Blobs can be sorted using the same Blob attributes as listed above.
                 * No more than one sort call should be made.  Sorting can use ascending or descending order.
                 *     ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);      // Default
                 *     ColorBlobLocatorProcessor.Util.sortByDensity(SortOrder.DESCENDING, blobs);
                 *     ColorBlobLocatorProcessor.Util.sortByAspectRatio(SortOrder.DESCENDING, blobs);
                 */
                ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING,blobs);


                // Display the size (area) and center location for each Blob.
                ColorBlobLocatorProcessor.Blob b = blobs.get(0);
                RotatedRect boxFit = b.getBoxFit();
                PoseStorage.grabColorPose = new Pose2d(boxFit.center.x,boxFit.center.y,Math.toRadians(90));
                RobotLog.dd("SUB GRAB POSE", PoseStorage.grabColorPose.position.x+","+PoseStorage.grabColorPose.position.y+","+PoseStorage.grabColorPose.heading.toDouble());
                return false;
            }
        }
    }
    public Action comp (double tim) {
        return new computeTargetPose(tim);
    }
}
