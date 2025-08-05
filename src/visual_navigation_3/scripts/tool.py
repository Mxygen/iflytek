import cv2


Dir = 1    #1 for Left   -1 for Right
Line = []


def main():
    cap = cv2.VideoCapture(0)
    cv2.namedWindow("GaussFrame")
    while True:
        ret, frame = cap.read()
        frame = cv2.flip(frame, 1)
        frame = cv2.resize(frame, (160,120))
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # frame = cv2.GaussianBlur(frame, (3,3), 1.5)
        frame = cv2.Canny(frame,50,90)
        displayFrame = searchLine(frame)
        key = cv2.waitKey(1)
        if key == ord("s"):
            save(Dir)
        elif key == ord("q"):
            break
        displayFrame = cv2.resize(frame, (640,480))
        cv2.imshow("GaussFrame",displayFrame)
        Line.clear()
    

    cap.release()
    cv2.destroyAllWindows()



def searchLine(frame):
    global Line
    Stop = 0

    displayFrame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    for i in range(119,0,-1):
        if Stop:
            break

        found = False
        if Dir == 1:
            for j in range(159):
 
                if frame[i][j]:
                    if j < 3:
                        Stop = 1
                        break
                    Line.append(j)
                    found = True
                    break
        elif Dir == -1:
            for j in range(159,0,-1):
                if frame[i][j]:
                    if j >157:
                        Stop = 1
                        break
                    Line.append(j)
                    found = True
                    break

        if not found:
            if Dir == 1:
                Line.append(159)
            elif Dir == -1:
                Line.append(0)
        else:
            displayFrame[i][Line[-1]] = [0,0,255]
    

    return displayFrame
            



def save(dir):

    if dir == 1:
        fileHead = f"int mask_L[{len(Line)}] = {{ \n"
        fileEnd = " };"
        other = f"int maskLen_L = {len(Line)} ;\n" + \
                f"extern int maskLen_L; \n" + \
                f"extern int mask_L[{len(Line)}];"
        with open("/home/ucar/ucar_ws/src/visual_navigation_3/include/mask_L.h","w") as file:
            file.write(fileHead)
            for i in range(len(Line)-1,-1,-1):
                file.write(f"{Line[i]}" + " ,")
                if i % 10 == 0:
                    file.write("\n")
            file.write(fileEnd + "\n")
            print(f"Line Save to mask_L.h  total lenth: {len(Line)}")
            file.write(other)

            
    elif dir == -1:
        fileHead = f"int mask_R[{len(Line)}] = {{ \n"
        fileEnd = " };"
        other = f"int maskLen_R = {len(Line)} ;\n" + \
                f"extern int maskLen_R; \n" + \
                f"extern int mask_R[{len(Line)}];"

        with open("/home/ucar/ucar_ws/src/visual_navigation_3/include/mask_R.h","w") as file:
            file.write(fileHead)
            for i in range(len(Line)-1,-1,-1):
                file.write(f"{Line[i]}" + " ,")
                if i % 10 == 0:
                    file.write("\n")
            file.write(fileEnd + "\n")
            print(f"Line Save to mask_R.h  total lenth: {len(Line)}")
            file.write(other)



if __name__ == "__main__":
    main()