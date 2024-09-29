import torch
torch.set_num_threads(1)

from IPython.display import Audio
from pprint import pprint
# download example
torch.hub.download_url_to_file('https://models.silero.ai/vad_models/en.wav', 'en_example.wav')

model, utils = torch.hub.load(repo_or_dir='snakers4/silero-vad',
                              model='silero_vad',
                              force_reload=True)

(get_speech_timestamps,
 _, read_audio,
 *_) = utils

sampling_rate = 16000 # also accepts 8000
wav = read_audio('en_example.wav', sampling_rate=sampling_rate)
# get speech timestamps from full audio file
speech_timestamps = get_speech_timestamps(wav, model, sampling_rate=sampling_rate)
pprint(speech_timestamps)
