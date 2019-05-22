import cv2

if __name__ == "__main__":
    print "hello world"
    cap = cv2.VideoCapture('http://192.168.8.102:9000/?action=stream')
    while True:
        ret, frame = cap.read()
        cv2.imshow('Video', frame)

        if cv2.waitKey(1) == 27:
            exit(0)
