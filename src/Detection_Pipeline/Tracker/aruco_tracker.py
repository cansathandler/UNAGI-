import cv2
import numpy as np


class ArucoTracker:
    def __init__(
        self,
        camera_matrix,
        dist_coeffs,
        marker_length=0.05,
        dictionary=cv2.aruco.DICT_4X4_50,
    ):
        """
        marker_length: marker size in meters
        """

        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.marker_length = marker_length

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
                self.marker_length,
                self.camera_matrix,
                self.dist_coeffs,
            )

            for i, marker_id in enumerate(ids.flatten()):

                rvec = rvecs[i]
                tvec = tvecs[i]

                cv2.drawFrameAxes(
                    frame,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvec,
                    tvec,
                    self.marker_length * 0.6,
                )

                detections[int(marker_id)] = {
                    "corners": corners[i],
                    "rvec": rvec,
                    "tvec": tvec,
                }

                self.marker_operation(marker_id, rvec, tvec)

        return frame, detections

    def marker_operation(self, marker_id, rvec, tvec):
        """
        Override this function for custom operations.
        """

        x = float(tvec[0][0])
        y = float(tvec[0][1])
        z = float(tvec[0][2])

        print(
            f"Marker {marker_id}: "
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

    tracker = ArucoTracker(
        camera_matrix,
        dist_coeffs,
        marker_length=0.05,
    )

    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        raise RuntimeError("Cannot open camera.")

    while True:

        ret, frame = cap.read()

        if not ret:
            break

        frame, detections = tracker.process_frame(frame)

        cv2.imshow("Aruco Tracker", frame)

        key = cv2.waitKey(1)

        if key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()