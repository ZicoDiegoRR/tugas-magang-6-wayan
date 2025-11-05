import os
import sys
import cv2
import time
import rospy
from opencv_ros.srv import detection

# ROS Noetic doesn't allow imports normally like Python
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from detection_module import detect_all

def initiate_ros(colors, shapes):
    rospy.wait_for_service("process_image")
    try:
        client = rospy.ServiceProxy("process_image", detection)
        returned_shape = ", ".join([shape + "s" for shape in list(set(shapes))]) if shapes else ""

        response = client(returned_shape, colors[0], colors[1])
        return response.done
    except rospy.ServiceException as e:
        return f"Exception occured when contacting the server.\nError: {e}"

if __name__ == "__main__":
    rospy.init_node("image_processor")
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Couldn't open a camera at the moment.")
        exit()

    prev_time = 0
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            print("Failed to the read the image from the camera.")
            break

        resized = cv2.flip(cv2.resize(frame, (640, 480)), 1)

        now = time.time()
        fps = 1//(now - prev_time)
        prev_time = now

        resized, colors, shapes = detect_all(resized)
        result = initiate_ros(colors, shapes)
        cv2.putText(resized, str(fps), (10,32), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)

        cv2.imshow("Live Webcam", resized)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print("Stopped by user.")
            break
        if key == ord('s'):
            print("Saved")
            print(frame.shape)
            cv2.imwrite(f"/home/kali_zico/Desktop/Abhinara-1_Internship_Day6/py/source/{now}.png", resized)

    cap.release()
    cv2.destroyAllWindows()
