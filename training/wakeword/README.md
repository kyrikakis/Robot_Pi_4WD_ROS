# Wake word training ("hey rainbow", "stop", "hey robot")

openWakeWord models for `rainbow_voice` (`wake_models` / `stop_model`, shipped in
`src/rainbow_voice/models/`), trained on a laptop in Docker (CPU) from synthetic speech
only; no recordings needed.

Data (~38 GB) lives on the Docker volume `rainbow_wakeword` (`/data`): every round's
clips and features, so later rounds (and real recordings) reuse them. Keep that volume
(and a copy of it outside Docker: a Docker Desktop reset wipes volumes). The
Whisper-checked accent clips live in a host folder, `$POOL` below (`/pool`).

```
cd training/wakeword
docker build -t rainbow-wakeword .
POOL=/path/to/accent_pool
RUN="docker run --rm --shm-size=8g -v rainbow_wakeword:/data -v $POOL:/pool \
     -v $PWD:/src:ro -v $PWD/../../src/rainbow_voice/rainbow_voice:/robot:ro rainbow-wakeword"

# once: datasets (the two openWakeWord feature files, 17 GB + 185 MB, and the LibriTTS-R
# generator are fetched separately into /data/features and /data/models, see below)
$RUN python /src/prepare_data.py
# once per word: accented clips, then keep those Whisper hears right (MLX, the agent's venv)
$RUN python /src/make_accent_pool.py "hey rainbow"
~/.rainbow-agent/venv/bin/python whisper_filter.py $POOL
# a training round (clips and features are reused by later rounds; --shm-size: the
# default 64 MB kills PyTorch's DataLoader workers). Rounds take 6-10 h; run them
# detached (docker run -d) and keep the laptop awake and on its charger.
$RUN python /src/run_train.py /src/configs/hey_rainbow.yml --tag r1
# the name alone as well ("rainbow"), after augmentation, before training
$RUN python /src/add_word.py hey_rainbow rainbow
```

Downloads not done by `prepare_data.py` (into the volume):
`huggingface.co/datasets/davidscripka/openwakeword_features` (revision `985bf1b4`):
`openwakeword_features_ACAV100M_2000_hrs_16bit.npy` and `validation_set_features.npy`
into `/data/features/`; `en_US-libritts_r-medium.pt` from piper-sample-generator's v2.0.0
release (sha256 `e95ee537...0ff6`) into `/data/models/`.

- **Positives**: 70% LibriTTS-R generator (speakers 0-799 for training, 800-899 for
  testing, random blends), 30% accent voices (British, Scottish, non-native English,
  European/Indian voices reading English, Greek pronunciations) that Whisper heard as
  the phrase.
- **Negatives**: openWakeWord's 2,000 h of precomputed features (ACAV100M), generated
  near-miss phrases, and the config's custom ones (repeated to ~40% of the near misses).
  Texts containing the wake word itself are dropped (`WW_NEGATIVE_EXCLUDE`): "bus stop"
  as a negative taught the model not to fire at the end of "stop".
- **Augmentation**: AudioSet noise/music/speech, espeak speech (the robot's own voice),
  MIT and simulated far-field room impulse responses, extra rounds with more room and
  noise (`extra_positives.py`).
- **Eval** (`eval.py`): the robot's own `wakeword.py`, streaming in 80 ms chunks with
  a 2 s cooldown: recall on unseen voices in silence / a quiet room / 10/5/0 dB / far
  across a room (by voice), near-miss fires, false accepts per hour on held-out AudioSet
  (~5.5 h) and on espeak. `/data/results/<model>/<tag>/` keeps each round's config,
  model and numbers.

## Results (2026-11)

Synthetic test voices at the chosen thresholds; "hey Jarvis" (openWakeWord's) scores
99/95/89/71% in quiet/10/5/0 dB and 78% across the room.

| Model | Round | Threshold | Quiet | 10/5/0 dB | Room | False/h |
|---|---|---|---|---|---|---|
| hey_rainbow ("hey Rainbow" / "Rainbow") | r8 | 0.3 | 100% | 84/69/42% | 45% | 0.7 |
| stop | r4 (layer 128) | 0.7 | 78% | 60/49/36% | 36% | 2 |
| hey_robot ("hey Robot" / "Robot") | r2 | 0.3 | 82% | 72/62/47% | 42% | 0.4 |

On the robot (the owner, through the ReSpeaker): "hey Rainbow" works far better than
"hey Jarvis" did; "stop" scores 0.5-0.9; "Robot" stays under 0.3, so `hey_robot` ships
but is not in `wake_models`. Synthetic training plateaued below "hey Jarvis" in noise.

## Next, when wanted

- **The robot's own noise**: the lidar's hum and the Pi's fan, turned up by the
  ReSpeaker's automatic gain whenever nobody speaks. Record a few minutes of it (nobody
  talking; stop `rainbow_voice` first, `arecord -D plughw:CARD=Lite,DEV=0 -f S16_LE -r
  16000 -c 2`), add it to the background paths and re-augment, then retrain `stop` and
  `hey_robot` (training only: ~3.5 h per model) and evaluate in it.
- **The household's own voices**: training does not start over. Record 20-50 clips per
  person per phrase through the ReSpeaker, at varied distances, loudness and speed, plus
  a few minutes of household talk/TV without the words. Augment them in several rounds
  like `add_word.py` does, repeat them so they weigh enough against 15-30k synthetic
  clips, append to `positive_features_train.npy` (talk: to the negatives), and run only
  `--steps train,eval`. Even before retraining, evaluating the models on those
  recordings gives the household's real recall.
