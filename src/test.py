import cv2
import numpy as np

def line_symmetry(frame, camera):
    cv2.rectangle(frame, (635, 0), (645, 720), (0, 255, 0), 2, cv2.LINE_4)
    cv2.rectangle(frame, (0, 680), (1280, 690), (0, 255, 0), 2, cv2.LINE_4)
    cv2.rectangle(frame, (0, 640), (1280, 650), (0, 255, 0), 2, cv2.LINE_4)
    cv2.rectangle(frame, (0, 600), (1280, 610), (0, 255, 0), 2, cv2.LINE_4)
    cv2.rectangle(frame, (0, 560), (1280, 570), (0, 255, 0), 2, cv2.LINE_4)
    cv2.rectangle(frame, (0, 520), (1280, 530), (0, 255, 0), 2, cv2.LINE_4)
    cv2.rectangle(frame, (0, 480), (1280, 490), (0, 255, 0), 2, cv2.LINE_4)
    cv2.rectangle(frame, (0, 360), (1280, 370), (0, 255, 0), 2, cv2.LINE_4)

    if camera == 0:
        cv2.imshow("left_line_symmetry", frame)
    else:
        cv2.imshow("right_line_symmetry", frame)


def add_hsv_filter(frame, camera):
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = np.zeros_like(hsv_frame)

    # logitech c930e
    left_lowerYellow = np.array([10, 160, 100])  # Lower limit for yellow
    left_upperYellow = np.array([40, 255, 255])  # Upper limit for yellow
    right_lowerYellow = np.array([10, 160, 100])  # Lower limit for yellow
    right_upperYellow = np.array([40, 255, 255])  # Upper limit for yellow

    if camera == 0:
        mask = cv2.inRange(hsv_frame, left_lowerYellow, left_upperYellow)
        cv2.imshow("left", mask)
    else:
        mask = cv2.inRange(hsv_frame, right_lowerYellow, right_upperYellow)
        cv2.imshow("right", mask)

# 첫 번째 웹캠에 대한 연결 생성
cap1 = cv2.VideoCapture(0)  # /dev/video0

# 두 번째 웹캠에 대한 연결 생성
cap2 = cv2.VideoCapture(4)  # /dev/video1

while True:
    # 첫 번째 웹캠 프레임 읽기
    ret1, frame1 = cap1.read()
    if not ret1:
        break

    # 두 번째 웹캠 프레임 읽기
    ret2, frame2 = cap2.read()
    if not ret2:
        break

    frame1 = cv2.resize(frame1, (1280,720))
    frame2 = cv2.resize(frame2, (1280,720))

    add_hsv_filter(frame1, 0)
    add_hsv_filter(frame1, 4)
    #cv2.imshow("Webcam 1", frame1)
    #cv2.imshow("Webcam 2", frame2)

    # 종료 조건 설정
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 연결 해제
cap1.release()
cap2.release()
cv2.destroyAllWindows()
