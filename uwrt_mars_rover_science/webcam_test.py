import cv2

# 0 is default index of webcam
vid = cv2.VideoCapture(0)

while(True):
    # read frame
    ret, frame = vid.read()
    # display frame
    cv2.imshow('frame', frame)

    # press q to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

vid.release()
cv2.destroyAllWindows()
