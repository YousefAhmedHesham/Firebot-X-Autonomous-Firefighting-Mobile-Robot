import cv2

# Open video capture
cap = cv2.VideoCapture(0)

# Set camera resolution 
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)




while True:
    success, img = cap.read()
    if not success:
        break

    img = cv2.flip(img, 1)  # Flip image horizontally

   # img = cv2.rotate(img, cv2.ROTATE_90)
    cv2.imshow('camera', img)
    


cap.release()
cv2.destroyAllWindows()