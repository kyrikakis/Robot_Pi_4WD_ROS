# Wake word training ("hey rainbow", "stop")

openWakeWord models for `rainbow_voice` (`wake_model` / `stop_model`), trained on the
Mac in Docker (CPU) from synthetic speech only; no recordings needed.

Data (~35 GB) lives on the Docker volume `rainbow_wakeword` (`/data`); the Whisper-checked
accent clips live on the Mac in `~/rainbow-wakeword-data/accent_pool` (`/pool`).

```
cd training/wakeword
docker build -t rainbow-wakeword .
RUN="docker run --rm --shm-size=8g -v rainbow_wakeword:/data -v ~/rainbow-wakeword-data/accent_pool:/pool \
     -v $PWD:/src:ro -v $PWD/../../src/rainbow_voice/rainbow_voice:/robot:ro rainbow-wakeword"

# once: datasets (the two openWakeWord feature files, 17 GB + 185 MB, and the LibriTTS-R
# generator are fetched separately into /data/features and /data/models, see below)
$RUN python /src/prepare_data.py
# once per word: accented clips, then keep those Whisper hears right (on the Mac, MLX)
$RUN python /src/make_accent_pool.py "hey rainbow"
~/.rainbow-agent/venv/bin/python whisper_filter.py ~/rainbow-wakeword-data/accent_pool
# a training round (clips and features are reused by later rounds; --shm-size: the
# default 64 MB kills PyTorch's DataLoader workers)
$RUN python /src/run_train.py /src/configs/hey_rainbow.yml --tag r1
```

Downloads not done by `prepare_data.py` (into the volume):
`huggingface.co/datasets/davidscripka/openwakeword_features` (revision `985bf1b4`):
`openwakeword_features_ACAV100M_2000_hrs_16bit.npy` and `validation_set_features.npy`
into `/data/features/`; `en_US-libritts_r-medium.pt` from piper-sample-generator's v2.0.0
release (sha256 `e95ee537...0ff6`) into `/data/models/`.

- **Positives**: 70% LibriTTS-R generator (speakers 0-799 for training, 800-899 for
  testing, random blends), 30% accent voices (British, Scottish, non-native English, and
  European/Indian voices reading English) that Whisper heard as the phrase.
- **Negatives**: openWakeWord's 2,000 h of precomputed features (ACAV100M), generated
  near-miss phrases, and the config's custom ones (repeated to ~40% of the near misses).
- **Augmentation**: AudioSet noise/music/speech, espeak speech (the robot's own voice),
  MIT room impulse responses.
- **Eval** (`eval.py`): the robot's own `wakeword.py`, streaming in 80 ms chunks with
  a 2 s cooldown: recall on unseen voices at clean/10/5/0 dB SNR (by voice), near-miss
  fires, false accepts per hour on held-out AudioSet (~5.5 h) and on espeak.
  `/data/results/<model>/<tag>/` keeps each round's config, model and numbers.

## Later: the household's own voices

Training does not start over. Each round keeps its clips and features in the Docker volume
`rainbow_wakeword` (38 GB; copied to `~/rainbow-wakeword-data/volume-backup/`, and the
stopped container `ww_keep_data` keeps it in use so a prune skips it). To add real voices:
record 20-50 clips per person per phrase, best through the robot's ReSpeaker, at varied
distances, loudness and speed, plus a few minutes of household talk/TV without the words.
Augment them in several rounds like `add_word.py` does, repeat them so they weigh enough
against 15-30k synthetic clips, append to `positive_features_train.npy` (talk: to the
negatives), and run only `--steps train,eval` (~3.5 h per model). Even before retraining,
evaluating the models on those recordings gives the household's real recall.
