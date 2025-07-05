import cv2

def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open video.")
        return

    while True:
        ret, frame = cap.read()
        frame = cv2.resize(frame, (160,120))  # Resize to 320x240
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert to grayscale
        frame = cv2.GaussianBlur(frame, (5, 5), 1.5)
        frame = cv2.Canny(frame, 45, 95)  # Apply C
        if not ret:
            print("Error: Could not read frame.")
            break

        frame = cv2.flip(frame, 1)
        cv2.imshow("Video Feed", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()