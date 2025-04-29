from flask import Flask, render_template, request, redirect, url_for, send_from_directory, Response
from ultralytics import YOLO
import cv2
import os
from werkzeug.utils import secure_filename

app = Flask(__name__)

# Load your trained YOLOv8 model
model = YOLO("trained_tomato_yolov8.pt")

# Directory to store uploaded images
UPLOAD_FOLDER = 'static/uploads'
os.makedirs(UPLOAD_FOLDER, exist_ok=True)
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER

# Home route – Upload & live options
@app.route('/', methods=['GET', 'POST'])
def index():
    if request.method == 'POST':
        image_file = request.files['image']
        if image_file:
            filename = secure_filename(image_file.filename)
            image_path = os.path.join(app.config['UPLOAD_FOLDER'], filename)
            image_file.save(image_path)

            # Perform detection
            results = model(image_path)[0]
            img = cv2.imread(image_path)

            for box in results.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                label = f"{model.names[cls_id]}: {conf:.2f}"
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Save result image
            result_path = os.path.join(app.config['UPLOAD_FOLDER'], 'result_' + filename)
            cv2.imwrite(result_path, img)

            return render_template('index.html', result_image=result_path)

    return render_template('index.html')

# Webcam route
def generate_camera_stream():
    cap = cv2.VideoCapture(0)
    while True:
        success, frame = cap.read()
        if not success:
            break

        results = model(frame)[0]
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            label = f"{model.names[cls_id]}: {conf:.2f}"
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        _, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

    cap.release()

@app.route('/live')
def live():
    return Response(generate_camera_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    app.run(debug=True)
