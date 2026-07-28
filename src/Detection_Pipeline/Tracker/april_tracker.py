import cv2
import numpy as np


class AprilTagTracker:
    def __init__(
        self,
        camera_matrix,
        dist_coeffs,
        tag_length=0.05,
        dictionary=cv2.aruco.DICT_APRILTAG_36h11,
    ):
        """
        tag_length: tag size in meters
        """

        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.tag_length = tag_length

        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary)

        self.parameters = cv2.aruco.DetectorParameters()

        self.detector = cv2.aruco.ArucoDetector(
            self.dictionary,
            self.parameters,
        )

    def process_frame(self, frame):

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = self.detector.detectMarkers(gray)

        detections = {}

        if ids is not None:

            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners,
                self.tag_length,
                self.camera_matrix,
                self.dist_coeffs,
            )

            for i, tag_id in enumerate(ids.flatten()):

                rvec = rvecs[i]
                tvec = tvecs[i]

                cv2.drawFrameAxes(
                    frame,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvec,
                    tvec,
                    self.tag_length * 0.6,
                )

                detections[int(tag_id)] = {
                    "corners": corners[i],
                    "rvec": rvec,
                    "tvec": tvec,
                }

                self.tag_operation(tag_id, rvec, tvec)

        return frame, detections

    def tag_operation(self, tag_id, rvec, tvec):

        x = float(tvec[0][0])
        y = float(tvec[0][1])
        z = float(tvec[0][2])

        print(
            f"AprilTag {tag_id}: "
            f"x={x:.3f} "
            f"y={y:.3f} "
            f"z={z:.3f}"
        )


def main():

    #############################################
    # REPLACE WITH YOUR CAMERA CALIBRATION
    #############################################

    camera_matrix = np.array([
        [900,   0, 640],
        [0,   900, 360],
        [0,     0,   1],
    ], dtype=np.float32)

    dist_coeffs = np.zeros((5, 1))

    #############################################

    tracker = AprilTagTracker(
        camera_matrix,
        dist_coeffs,
        tag_length=0.05,
        dictionary=cv2.aruco.DICT_APRILTAG_36h11,
    )

    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        raise RuntimeError("Cannot open camera.")

    while True:

        ret, frame = cap.read()

        if not ret:
            break

        frame, detections = tracker.process_frame(frame)

        cv2.imshow("AprilTag Tracker", frame)

        if cv2.waitKey(1) == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()