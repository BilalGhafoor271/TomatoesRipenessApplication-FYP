import cv2
import numpy as np

cap = cv2.VideoCapture(0)
clicked_points = []

def click_event(event, x, y, flags, params):
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_points.append([x, y])
        print(f"Clicked: {x}, {y}")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    cv2.imshow("Click 4 corners (TL, TR, BR, BL)", frame)
    cv2.setMouseCallback("Click 4 corners (TL, TR, BR, BL)", click_event)

    if len(clicked_points) == 4:
        break

    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()

print("Pixel points:", clicked_points)
