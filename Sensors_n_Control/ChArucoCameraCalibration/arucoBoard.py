'''THIS CODE IS BASED OFF THE CODE FROM https://github.com/nullboundary/CharucoCalibration
IT HAS BEEN PORTED TO PYTHON 3.8
Environment
opencv-contrib-python                4.5.5.62
opencv-python                        4.8.0.76
numpy                                1.24.4
Run the following command in the directory:
python3 arucoBoard.py  -f calibrated.yml 10.jpg  11.jpg  12.jpg  13.jpg  14.jpg  15.jpg  16.jpg  17.jpg  18.jpg  19.jpg  1.jpg  20.jpg  21.jpg  22.jpg  23.jpg  24.jpg  2.jpg  3.jpg  4.jpg  5.jpg  6.jpg  7.jpg  8.jpg  9.jpg
'''
import numpy as np
import cv2
import cv2.aruco as aruco
import argparse
import sys
import yaml

def saveCameraParams(filename, imageSize, cameraMatrix, distCoeffs, totalAvgErr):

    print(cameraMatrix)

    calibration = {
        'camera_matrix': cameraMatrix.tolist(),
        'distortion_coefficients': distCoeffs.tolist()
    }

    calibrationData = {
        'image_width': imageSize[1],  # Note the swap here
        'image_height': imageSize[0],  # Note the swap here
        'camera_matrix': {
            'rows': cameraMatrix.shape[0],
            'cols': cameraMatrix.shape[1],
            'dt': 'd',
            'data': cameraMatrix.tolist(),
        },
        'distortion_coefficients': {
            'rows': distCoeffs.shape[0],
            'cols': distCoeffs.shape[1],
            'dt': 'd',
            'data': distCoeffs.tolist(),
        },
        'avg_reprojection_error': totalAvgErr,
    }

    with open(filename, 'w') as outfile:
        yaml.dump(calibrationData, outfile)

parser = argparse.ArgumentParser()

parser.add_argument("-f", "--file", help="output calibration filename", default="calibration.yml")
parser.add_argument("-s", "--size", help="size of squares in meters", type=float, default=0.019)
parser.add_argument('imgs', nargs='+', help='list of images for calibration')
args = parser.parse_args()

sqWidth = 11  # number of squares width
sqHeight = 8  # number of squares height
allCorners = []  # all Charuco Corners
allIds = []  # all Charuco Ids
decimator = 0
checkerSize = 0.019 
markerSize = 0.0145

dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
board = aruco.CharucoBoard_create(sqWidth, sqHeight, checkerSize, markerSize, dictionary)

for f in args.imgs:
    print(f"reading {f}")
    img = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
    markerCorners, markerIds, rejectedImgPoints = aruco.detectMarkers(img, dictionary)

    if len(markerCorners) > 0:
        ret, charucoCorners, charucoIds = aruco.interpolateCornersCharuco(markerCorners, markerIds, img, board)
        if charucoCorners is not None and charucoIds is not None and len(charucoCorners) > 3 and decimator % 3 == 0:
            allCorners.append(charucoCorners)
            print(allCorners)
            allIds.append(charucoIds)

        aruco.drawDetectedMarkers(img, markerCorners, markerIds)
        aruco.drawDetectedCornersCharuco(img, charucoCorners, charucoIds)

    smallimg = cv2.resize(img, (1024, 768))
    cv2.imshow("frame", smallimg)
    cv2.waitKey(0)
    decimator += 1

imsize = img.shape[:2]  # Height x Width

print(imsize)

# Try Calibration
try:
    ret, cameraMatrix, disCoeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(allCorners, allIds, board, imsize, None, None)
    print(f"Rep Error: {ret}")
    saveCameraParams(args.file, imsize, cameraMatrix, disCoeffs, ret)

except ValueError as e:
    print(e)
except NameError as e:
    print(e)
except AttributeError as e:
    print(e)
except Exception as e:
    print(f"calibrateCameraCharuco fail: {e}")

print("Press any key on window to exit")
cv2.waitKey(0)
cv2.destroyAllWindows()