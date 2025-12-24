import cv2
import threading
import time
from flask import Flask, Response
from flask_cors import CORS

# Flask app setup
app = Flask(__name__)
CORS(app)  # Enable CORS to allow access from other devices

# Video capture settings
cap = cv2.VideoCapture(0)  # Use the nth webcam
#ap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
# cap.set(cv2.CAP_PROP_FPS, 30)

# Global variables
frame = None
lock = threading.Lock()
fps = 20


def capture_frames():
    """ Continuously capture frames from the webcam """
    # global frame  # not needed - only if you write to it
    while True:
        ret, img = cap.read()
        if ret:
            with lock:
                frame = img
        time.sleep(1 / fps)  # Maintain 30 FPS


@app.route('/stream.mjpg')
def stream():
    """ Stream video frames as an MJPEG feed """

    def generate():
        # global frame  # not needed - only if you write to it
        while True:
            with lock:
                if frame is None:
                    continue
                _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' +
                   buffer.tobytes() + b'\r\n')

    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    # Start the webcam thread
    threading.Thread(target=capture_frames, daemon=True).start()

    # Start the Flask server
    app.run(host='127.0.0.1', port=1186, debug=False, threaded=True)
	
