import cv
retval, corners = cv2.findChessboardCorners(image, patternSize, flags)