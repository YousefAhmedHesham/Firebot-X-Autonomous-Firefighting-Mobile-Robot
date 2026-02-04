import cv2  # Import the OpenCV library for computer vision tasks
# Load the fire detection classifier from an XML file (must be trained or pre-trained properly)
# This file should contain a Haar cascade or LBP classifier specifically trained to detect fire
fire_detector = cv2.CascadeClassifier("fire_detection.xml")
# Open the default camera (camera index 0). This initializes the video capture stream.
cap = cv2.VideoCapture(0)
# Start an infinite loop to read and process video frames in real time
while True:
    # Read a single frame from the video capture object
    success, img = cap.read()

    # If the frame was not successfully read (e.g., webcam not connected), exit the loop
    if not success:
        break

    # Flip the captured frame horizontally to create a mirror image
    # This makes the video feed more intuitive for the user (like a selfie view)
    img = cv2.flip(img, 1)

    # Convert the color frame to grayscale
    # The detection algorithm works on grayscale images for simplicity and performance
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


    # Detect fire in the grayscale image using the loaded cascade classifier
    # Parameters: 
    #   scaleFactor=1.2 - how much the image size is reduced at each image scale
    #   minNeighbors=5 - how many neighbors each candidate rectangle should have to retain it
    fire = fire_detector.detectMultiScale(gray, 1.2, 5)

    # Iterate over all detected fire regions (if any)
    for (x, y, w, h) in fire:
        # Draw a red rectangle around the detected fire region on the (modified) original image
        # Parameters: image, top-left corner, bottom-right corner, color (BGR), thickness
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 3)

        # Print a message to the console when fire is detected
        print("Fire detected")
    # Display the processed image with detection annotations in a window titled 'Fire Detection'
    cv2.imshow('Fire Detection', img)

    # Wait for 1 ms for a key press
    # If the 'q' key is pressed, break the loop and exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
# Release the camera resource and stop the video stream
cap.release()

# Close all OpenCV windows that were opened
cv2.destroyAllWindows()
