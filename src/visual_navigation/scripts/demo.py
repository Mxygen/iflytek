import cv2

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    # frame = cv2.resize(frame, (160, 120))
    frame = cv2.flip(frame, 1)
    # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # frame = cv2.GaussianBlur(frame, (5, 5), 1.5)
    # frame = cv2.Canny(frame, 50, 90)
    cv2.imshow("frame", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()