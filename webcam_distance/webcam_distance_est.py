import cv2

#distance from camera to object(face) measured in cm
Known_distance = 20

#width of face in the real world or Object Plane in cm
Known_width = 6.7

GREEN = (0, 255, 0)
RED = (0, 0, 255)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)

fonts = cv2.FONT_HERSHEY_COMPLEX

face_detector = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")


#Find focal length
def Focal_Length_Finder(measured_distance, real_width, width_in_rf_image):
    focal_length = (width_in_rf_image * measured_distance) / real_width
    return focal_length


#Estimate distance
def Distance_finder(Focal_Length, real_face_width, face_width_in_frame):
    distance = (real_face_width * Focal_Length) / face_width_in_frame

    return distance


def face_data(image):
    face_width = 0

    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    faces = face_detector.detectMultiScale(gray_image, 1.3, 5)

    #looping through the faces detect in the image
    #getting coordinates x, y , width and height
    for (x, y, h, w) in faces:
        cv2.rectangle(image, (x, y), (x + w, y + h), GREEN, 2)

        face_width = w

    return face_width


ref_image = cv2.imread("Ref_image.png")

ref_image_face_width = face_data(ref_image)

Focal_length_found = Focal_Length_Finder(
    Known_distance, Known_width, ref_image_face_width)

print(Focal_length_found)

cv2.imshow("ref_image", ref_image)


cap = cv2.VideoCapture(0)


while True:

    _, frame = cap.read()


    face_width_in_frame = face_data(frame)


    if face_width_in_frame != 0:

        Distance = Distance_finder(
            Focal_length_found, Known_width, face_width_in_frame)

        cv2.line(frame, (30, 30), (230, 30), RED, 32)
        cv2.line(frame, (30, 30), (230, 30), BLACK, 28)

        cv2.putText(
            frame, f"Distance: {round(Distance, 2)} CM", (30, 35),
            fonts, 0.6, GREEN, 2)

    cv2.imshow("frame", frame)

    if cv2.waitKey(1) == ord("q"):
        break

cap.release()

cv2.destroyAllWindows()