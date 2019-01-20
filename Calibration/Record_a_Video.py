import cv2

cap = cv2.VideoCapture(0)
fourcc = cv2.VideoWriter_fourcc(*"MJPG")
out = cv2.VideoWriter('./output.avi',fourcc, 20.0, (480,640))

if __name__ == '__main__':    
    while True:
        ret, frame = cap.read()
        frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        cv2.imshow('frame',frame)
        out.write(frame)
        if 27 == cv2.waitKey(50):
            break
    cap.release()
    out.release()