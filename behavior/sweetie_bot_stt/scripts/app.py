#!/usr/bin/env python3
from flask import Flask, request, redirect
import logging
from werkzeug.utils import secure_filename
import os
import time
import tempfile
from faster_whisper import WhisperModel

ALLOWED_EXTENSIONS = {'wav', 'mp3', 'ogg'}
compute_type='int8'
model='medium'
audio_model = model = WhisperModel(model, device="cuda", compute_type=compute_type)

ret = { "status": "", "text": "", "language": "", "transcribe_duration": 0, "audio_duration": 0 }

def allowed_file(filename):
    return '.' in filename and \
           filename.rsplit('.', 1)[1].lower() in ALLOWED_EXTENSIONS

app = Flask(__name__)
environment_configuration = os.environ.get('CONFIGURATION_SETUP', 'config.DevelopmentConfig')
app.config.from_object(environment_configuration)
app.logger.setLevel(logging.INFO)
app.logger.info(f"Debug: {app.config['DEBUG']}")

@app.route("/", methods=["GET", "POST"])
def create_binary():
    if request.method == 'POST':
        if request.content_type:
          app.logger.debug("Content type: %s", request.content_type.split(";")[0])
        if request.content_length:
          app.logger.debug("Content length: %s", request.content_length)
        if request.content_type.startswith("multipart/form-data"):
            file = request.files.get("file")
            if file is None:
                return ret, "400 Error! Wrong form-data type"
            if not allowed_file(file.filename):
                return ret, "400 Error! Wrong file type"
            file = request.files['file']
            temp_dir = tempfile.mkdtemp()
            temp_file= os.path.join(temp_dir, "audio.wav")
            file.save(temp_file)
            file.seek(0, os.SEEK_END)
            l = file.tell()
        else:  # if request.content_type == "application/octet-stream":
            data = request.get_data()
            if not data:
                return ret, "400 Error! No data"
            l = len(data)
            if l == 0:
                return ret, "400 Error! Empty data"

            temp_dir = tempfile.mkdtemp()
            temp_file= os.path.join(temp_dir, "audio.wav")
            with open(temp_file, "wb") as file:
                file.write(data)

        app.logger.debug("Length of input data: %d", l)
        start = time.time()
        try:
          segments, result = audio_model.transcribe(temp_file)
        except:
          return ret, "400 Error! Invalid data"
        transcribe_duration = time.time() - start
        app.logger.debug("Transcribtion result: %s", str(result))
        os.remove(temp_file)
        os.rmdir(temp_dir)
        for segment in segments:
            app.logger.info("Transcribed text: %s", segment.text.strip())
            return {
                    "status": "ok",
                    "text": segment.text.strip(),
                    "language": result.language,
                    "transcribe_duration": transcribe_duration,
                    "audio_duration": result.duration
                   }

        return {
                "status": "no segments",
                "text": "",
                "language": result.language,
                "transcribe_duration": transcribe_duration,
                "audio_duration": result.duration
               }

    # GET
    return f'''
    <!doctype html>
    <title>Sweetie Bot Transcribe API</title>
    <h1>Sweetie Bot Transcribe API</h1>
    <p>curl -F file=@file.wav {request.base_url}</p>
    <form method=post enctype=multipart/form-data>
      <input type=file name=file>
      <input type=submit value=Upload>
    </form>
    </html>
    '''

if __name__ == "__main__":
    app.run()
