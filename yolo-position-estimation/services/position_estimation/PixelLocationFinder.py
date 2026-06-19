import math

EARTH_RADIUS_M = 6378137.0


def computeGroundIntersection(u, v, h, thetaDeg, fovDeg, imgWidth, imgHeight):
    """
    Calculates the intersection between a ray passing through pixel (u, v) and the ground.

    The coordinate frame is defined as follows:
      - World: X = horizontal (east), Y = vertical (positive upward), Z = forward (north).
      - The camera is at (0, h, 0).
      - The ground is at Y = 0.

    The standard camera coordinate system is:
      - Origin at the center of the lens,
      - x: right, y: down, z: forward.

    To obtain a consistent transformation, pixel coordinates are converted to normalized
    coordinates, then a pitch rotation by angle alpha = -theta (in radians) is applied so that:
      - theta = 0 degrees means the camera looks horizontally (optical axis = (0, 0, 1))
      - theta = -90 degrees means the camera looks downward (optical axis = (0, -1, 0))

    Args:
        u, v: Pixel coordinates.
        h: Camera altitude in meters.
        thetaDeg: Camera pitch angle in degrees.
        fovDeg: Horizontal field of view in degrees.
        imgWidth, imgHeight: Image dimensions in pixels.

    Returns:
        (x, z): Ground coordinates relative to the camera's vertical projection.
                 Returns (None, None) if the ray does not intersect the ground.
    """
    # Camera intrinsic parameters.
    cx = imgWidth / 2.0
    cy = imgHeight / 2.0
    fov_h_rad = math.radians(fovDeg)
    fx = (imgWidth / 2.0) / math.tan(fov_h_rad / 2)
    fy = fx  # Assume square pixels.

    # Normalized coordinates in the camera frame.
    # Convert (u, v) to normalized (x, y) coordinates in the image plane.
    x = (u - cx) / fx
    y = (v - cy) / fy
    # Direction vector in camera coordinates.
    # Note: in the camera system, z=1 (focused distance).
    d_c = (x, y, 1.0)

    # Build the rotation matrix for pitch.
    # The target behavior is:
    #   - theta = 0 degrees gives R = identity (optical axis = (0, 0, 1))
    #   - theta = -90 degrees gives R * (0, 0, 1) = (0, -1, 0)
    # For this, use R = R_x(alpha) with alpha = -theta (in radians).
    alpha = math.radians(-thetaDeg)
    cos_alpha = math.cos(alpha)
    sin_alpha = math.sin(alpha)

    # Ray direction in the world frame.
    d_w = (
        d_c[0],
        cos_alpha * d_c[1] - sin_alpha * d_c[2],
        sin_alpha * d_c[1] + cos_alpha * d_c[2],
    )

    # Ray intersection with the ground (Y = 0).
    # P(t) = C + t * d_w; find t such that P_y = 0: h + t * (d_w)_y = 0.
    if d_w[1] >= 0:
        # The ray does not point down toward the ground.
        return math.inf, math.inf
    t = -h / d_w[1]
    # Return the ground coordinates (X, Z).
    return t * d_w[0], t * d_w[2]


def rotate_ne(forward, side, headingDegrees):
    heading_rad = -math.radians(headingDegrees)
    north = forward * math.cos(heading_rad) - side * math.sin(heading_rad)
    east = forward * math.sin(heading_rad) + side * math.cos(heading_rad)
    return north, east


def ned_to_lat_lon(northMeters, eastMeters, latitudeDegrees, longitudeDegrees):
    latitude_rad = math.radians(latitudeDegrees)
    latitude = latitudeDegrees + math.degrees(northMeters / EARTH_RADIUS_M)
    longitude = longitudeDegrees + math.degrees(
        eastMeters / (EARTH_RADIUS_M * math.cos(latitude_rad))
    )
    return latitude, longitude


class PixelLocationFinder:
    def __init__(
    self,
    nbPixHeight,
    nbPixWidth,
    fielfOfViewDegrees
    ):
        self.MAX_HEIGHT = nbPixHeight
        self.MAX_WIDTH = nbPixWidth
        self.FOV = fielfOfViewDegrees
        self.FOCAL_LENGTH_PIX = (
            self.MAX_WIDTH / 2.0 / math.tan(math.radians(self.FOV / 2))
        )

    def getRelativePositionFromPicture(self, xPix, yPix, cameraTiltDegrees, aglAltitudeMeters):
        py = self.MAX_HEIGHT - yPix
        side, forward = computeGroundIntersection(xPix, py, aglAltitudeMeters, cameraTiltDegrees, self.FOV, self.MAX_WIDTH, self.MAX_HEIGHT)
        return forward, side

    def getNePositionFromPicture(self, xPix, yPix, cameraTiltDegrees, aglAltitudeMeters, headingDegrees):
        forward, side = self.getRelativePositionFromPicture(xPix, yPix, cameraTiltDegrees, aglAltitudeMeters)
        N, E = rotate_ne(forward, side, headingDegrees)
        return N, E

    def getLatLonPositionFromPicture(self, xPix, yPix, cameraTiltDegrees, aglAltitudeMeters, headingDegrees, latitude, longitude):
        N, E = self.getNePositionFromPicture(xPix, yPix, cameraTiltDegrees, aglAltitudeMeters, headingDegrees)
        return ned_to_lat_lon(N, E, latitude, longitude)
    
    def computePixelLocations(self, corners):
        nedPos = {}
        for corner in corners:
            results = {}
            results["forward"], results["side"] = self.getRelativePositionFromPicture(corner[0], corner[1], CAMERA_TILT, AGL_ALT)
            results["N"], results["E"] = self.getNePositionFromPicture(corner[0], corner[1], CAMERA_TILT, AGL_ALT, HEADING)
            results["Lat"], results["Lon"] = self.getLatLonPositionFromPicture(corner[0], corner[1], CAMERA_TILT, AGL_ALT, HEADING, LAT, LON)

            nedPos[f'{corner[0]}_{corner[1]}'] = results
        for key, item in nedPos.items():
            print(f'Corner: {key}: N: {item}')

if __name__ == "__main__":
    # target=TargetFinder(2679, 554, 3456, 4608, -90, 0, 44.681817483345235 , -0.7100958485913744, 140, 70)
    pixPosition = [4608, 3456]
    MAX_WIDTH, MAX_HEIGHT = 4608, 3456
    FOV = 75.5
    corners = [[0, 0], [0, MAX_HEIGHT], [MAX_WIDTH, MAX_HEIGHT], [MAX_WIDTH, 0]]
    AGLAltitude = 120

    target = PixelLocationFinder(
        MAX_HEIGHT,
        MAX_WIDTH,
        FOV)

    CAMERA_TILT = -90+28.3
    AGL_ALT = 120
    HEADING = 90
    LAT, LON = 44.68, -0.70,
    nedPos = {}
    for corner in corners:
        results = {}
        results["forward"], results["side"] = target.getRelativePositionFromPicture(corner[0], corner[1], CAMERA_TILT, AGL_ALT)
        results["N"], results["E"] = target.getNePositionFromPicture(corner[0], corner[1], CAMERA_TILT, AGL_ALT, HEADING)
        results["Lat"], results["Lon"] = target.getLatLonPositionFromPicture(corner[0], corner[1], CAMERA_TILT, AGL_ALT, HEADING, LAT, LON)

        nedPos[f'{corner[0]}_{corner[1]}'] = results
    for key, item in nedPos.items():
        print(f'Corner: {key}: N: {item}')
