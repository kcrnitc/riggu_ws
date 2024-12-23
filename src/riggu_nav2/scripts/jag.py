import cv2
import dlib


detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor(dlib.shape_predictor_model_location())


registered_face_ids = set(['id1', 'id2', 'id3'])

def process_frame(frame, registered_face_ids):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = detector(gray)
    
    for face in faces:
        x, y, w, h = face.left(), face.top(), face.width(), face.height()
        
        face_id = 'unknown'

        color = (0, 255, 0) if face_id in registered_face_ids else (0, 0, 255)
        cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
    
    return frame

def main():
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = process_frame(frame, registered_face_ids)
        cv2.imshow('Face Detection', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()