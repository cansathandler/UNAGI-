import cv2
from pathlib import Path


def generate_apriltag(
    tag_id: int = 0,
    size: int = 600,
    filename: str = None,
    dictionary: int = cv2.aruco.DICT_APRILTAG_36h11,
):
    """
    Generate and save an AprilTag image.

    Args:
        tag_id: ID of the AprilTag.
        size: Image size in pixels.
        filename: Output filename. If None, uses tag_<id>.png.
        dictionary: AprilTag dictionary.
    """

    # Create tags folder in current working directory
    output_dir = Path.cwd() / "tags"
    output_dir.mkdir(exist_ok=True)

    if filename is None:
        filename = f"tag_{tag_id}_{size}.png"

    output_path = output_dir / filename

    aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary)

    marker = cv2.aruco.generateImageMarker(
        aruco_dict,
        tag_id,
        size,
    )

    cv2.imwrite(str(output_path), marker)

    print(f"Saved AprilTag {tag_id, size} to: {output_path}")


if __name__ == "__main__":
    generate_apriltag()