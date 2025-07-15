# vision.py
# -----------------------
# Functions for camera-based geometric estimation
# Used in object following drone for approximate distance calculation
# -----------------------

def estimate_distance_from_box(bbox_height, image_height):
    """
    Estimate the distance to an object using its bounding box height.

    Parameters:
    - bbox_height: Height of the detected bounding box (in pixels)
    - image_height: Height of the image frame (in pixels)

    Returns:
    - Estimated distance to the object (in meters)

    This uses a simple proportionality model:
        distance = k / bbox_height

    The constant k should be calibrated empirically for your specific:
    - Camera FOV
    - Resolution (640x480 here)
    - Object size (assumes person for MobileNet SSD)
    """

    if bbox_height <= 0:
        return 0.0

    # === Calibration constant ===
    # Adjust this based on tests:
    # For example, if bbox_height=200px at distance=6m:
    # k = 200 * 6 = 1200
    k = 1200

    est_distance = k / bbox_height

    # Optionally constrain distance range (safety filter)
    est_distance = max(0.3, min(est_distance, 10.0))  # Clamp between 0.3m and 10m

    return est_distance


def get_single_axis_delta(center_coordinate, target_coordinate):
    """
    Compute the pixel delta between image center and target center.

    Parameters:
    - center_coordinate: Coordinate of image center along an axis (x or y)
    - target_coordinate: Coordinate of target center along the same axis

    Returns:
    - Delta (positive if target is to the right/bottom, negative if left/top)
    """
    return target_coordinate - center_coordinate


