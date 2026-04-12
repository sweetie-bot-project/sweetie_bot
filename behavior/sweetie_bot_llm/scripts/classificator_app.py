#!/usr/bin/env python3
from flask import Flask, request, redirect
import logging
import sys
import os

from transformers import AutoModelForSequenceClassification, AutoConfig
from transformers import AutoTokenizer
import torch
import numpy as np

ALLOWED_EXTENSIONS = {'wav', 'mp3', 'ogg'}
compute_type=os.getenv('COMPUTE_TYPE', 'float16')
device=os.getenv('COMPUTE_DEVICE', 'cuda')

model_dir = os.getenv('MODEL_DIR', os.path.expanduser('~/repos/Bert-Base-Emotion-Sentiment-Analysis'))

ret = { "status": "", "class_probs_str": "" }

config = AutoConfig.from_pretrained(model_dir)

# config done
model = AutoModelForSequenceClassification.from_pretrained(model_dir, config=config).to(device)
model.eval()


app = Flask(__name__)
environment_configuration = os.environ.get('CONFIGURATION_SETUP', 'config.DevelopmentConfig')
app.config.from_object(environment_configuration)
app.logger.setLevel(logging.INFO)
app.logger.info(f"Debug: {app.config['DEBUG']}")

# Model loading
tokenizer = AutoTokenizer.from_pretrained(model_dir)

# def tokenize(batch):
#     return tokenizer(batch["text"], padding=True, truncation=True)

# TODO: Parametrize classes to general case
id2label= {
    0: "Sad",
    1: "Joy",
    2: "Love",
    3: "Anger",
    4: "Fear",
    5: "Surprise"
}

def parse_labels(class_probs: np.ndarray):
    class_probs = class_probs.flatten()
    assert class_probs.size == len(id2label.values()), "Number of classes must match label count"
    return {class_name: class_probs[class_id] for class_id, class_name in id2label.items()}


def str_json_compat(parsed_dict):
    s = str(parsed_dict)
    return s.replace("'", '"')

@app.route("/", methods=["GET", "POST"])
def get_classification_result():
    if request.method == 'POST':
        if request.content_type:
          app.logger.debug(f"Content type: {request.content_type.split(';')[0]}")
        if request.content_length:
          app.logger.debug(f"Content length: {request.content_length}")

        if request.is_json:
            input_text = request.json["text"]
        else:
            input_text = request.values.get("text")

        if input_text is None:
            ret["detail"] = "Input data is empty"
            return ret, f"400 Error! {ret['detail']}"

        app.logger.debug(f"Length of input text: {len(input_text)}")
        try:
            with torch.no_grad():
                inputs = tokenizer(input_text, return_tensors="pt")
                outputs = model(**{key: input.to(device) for key, input in inputs.items()})
                scores = torch.nn.Softmax(dim=1)(outputs.logits)
                result = scores.cpu().numpy()
        except Exception as e:
            ret["detail"] = "Invalid input data: {e}"
            return ret, f"400 Error! {ret['detail']}"

        app.logger.debug(f"Classification result: {str(result)}")
        app.logger.info(f"Most probable class: {id2label[torch.argmax(scores).item()]}")

        return {
                "status": "ok",
                "class_probs_str": str_json_compat(parse_labels(result)),
               }

    # GET
    return f'''
    <!doctype html>
    <title>Sweetie Bot Classification API</title>
    <h1>Sweetie Bot Classification API</h1>
    <p>curl -F text=<assesed text here> {request.base_url}</p>
    <form method=post enctype=multipart/form-data>
      <input type=text name=text>
      <input type=submit value=Classify>
    </form>
    </html>
    '''

if __name__ == "__main__":
    app.run(host=app.config['HOST'], port=app.config['PORT'])
