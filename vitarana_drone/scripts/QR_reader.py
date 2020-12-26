# QR code reader function
# input - image of QR code
# output - 3-tuple containing comma separated values from QR code

import cv2


def reader(image):
    img = cv2.imread(image)
    detector = cv2.QRCodeDetector()
    data, bbox, straight_qrcode = detector.detectAndDecode(img)
    coords = data.split(',')
    return coords


# image = 'C:\\Users\\anagh\\Downloads\\vitd.png'
# a = reader(image)
# print(a)


