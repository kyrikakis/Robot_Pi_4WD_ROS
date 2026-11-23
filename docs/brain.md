# Rainbow's brain: design

Status: agreed design, not built yet. Build order at the end.

Rainbow talks and acts on its own. You wake it by saying "Rainbow". A
conversational agent on the owner's MacBook (M4 Max) does the thinking: Whisper
for speech to text, plus a local vision-language model with tools and memory. A
local brain on the Pi owns the body: it runs the agent's requests, handles
simple commands and safety itself, and keeps going when the Mac or Wi-Fi is down.

## Principles

- **The Pi owns the body; the Mac only decides.** The Mac never publishes
  `cmd_vel` or talks to Nav2. It calls high-level skills, and one Pi node
  (`rainbow_brain`) decides what actually runs. Motion has a single owner.
- **Safety never crosses Wi-Fi.** "Stop", the collision monitor and the
  velocity watchdog all work with the Mac off.
- **Degrade, don't fail.** If the Mac drops, the robot finishes its current
  goal, takes no new decisions from the agent, and still understands a fixed set
  of spoken commands.
- **Always through Nav2.** The brain never publishes `cmd_vel`. Navigation is
  `NavigateToPose`; corner cases (turn to face someone, a short move) use Nav2's
  Spin and DriveOnHeading behaviours, so every move goes through the velocity
  smoother and the collision monitor. The brain never asks to drive backwards;
  only Nav2's own recovery may back up 10 cm (see AGENTS.md).
- **No ROS on the Mac.** ROS 2 on macOS is painful and DDS discovery over Wi-Fi
  is unreliable. The Mac talks to the Pi over one WebSocket.

## Overview

```
 ┌──────────────────────── Pi (robot container) ────────────────────────┐
 │ ESP / Nav2 / AMCL / ToF / head / eyes / speech   (existing)          │
 │        ▲ actions / services                                          │
 │ rainbow_brain                                                        │
 │   skills + mode (REMOTE / LOCAL) + router + places + link            │
 │        ▲ utterances (audio + local transcript)                       │
 │ rainbow_voice: ReSpeaker → openWakeWord → VAD → Vosk; espeak out     │
 └────────┼─────────────────────────────────────────────────────────────┘
          │ WebSocket, LAN only, token auth
 ┌────────▼──────────────── MacBook ────────────────────────────────────┐
 │ rainbow-agent (plain Python)                                         │
 │   Whisper (mlx-whisper) → Qwen3-VL on llama-server, with tools       │
 │   memory: SQLite + embeddings, rolling sessions                      │
 │   replies are text; the Pi speaks them with espeak                   │
 └──────────────────────────────────────────────────────────────────────┘
```

The Hailo container stays as it is, with face recognition re-enabled. It
provides still photos and recognised faces (see [Camera and faces](#camera-and-faces)).

## Voice (`rainbow_voice`)

Hardware: the ReSpeaker Lite (USB, 2 mics, speaker output) is both microphone
and speaker. ALSA card "Lite": 16 kHz, 16-bit, 2 channels; espeak plays on it
(`plughw:2,0`, as before). The voice node records by card name
(`plughw:CARD=Lite,DEV=0`, channel 0), since card numbers can change between
boots. Its two USB channels carry the same stream (measured: bit-identical);
the quiet-room noise floor is about -38 dBFS. English only for now.

Measured on the Pi: the wake word costs 3.8 ms per 80 ms chunk and the voice
node about 8% of one core while listening; Vosk takes ~80 ms per command; from
the wake word to the agent's spoken reply 2.6 s (0.8 s of it waiting for the
end of speech, Whisper 0.27 s, the model 0.3 s).

One node, `voice` (`voice_node.py`), owns the microphone (capture devices are
single-owner). It reads the card with `arecord` (no Python audio library) and
runs a state machine (`listener.py`) on every 80 ms chunk:

1. **Wake word**: openWakeWord's models, run on onnxruntime by our own
   `wakeword.py`. The openWakeWord package itself is not installed: it pulls in
   scipy and scikit-learn and wants a tflite-runtime that has no Python 3.12
   build. Its inference is small and reimplemented with the same models; the
   scores match the package's to 0.0001 (checked on test clips).
   **Placeholder:** the wake word is openWakeWord's pretrained "hey Jarvis"
   until a "Rainbow" model is trained (on the Mac, later). With a trained
   model, change `wake_model` (and a `stop_model`) in the node's parameters.
   A one-word wake word triggers falsely more often than a phrase; tune its
   threshold on recordings made with the motors running.
   **Training (not done yet)**: openWakeWord's training notebook
   (`notebooks/automatic_model_training.ipynb`) is Linux-only and pinned to
   Python 3.9-era packages (tensorflow-cpu 2.8.1 has no Apple Silicon build;
   TensorFlow is only for the optional .tflite export, we use .onnx). It
   generates the positive clips with piper-sample-generator
   (`en_US-libritts_r-medium.pt`, 204 MB, sha256 e95ee537...0ff6), downloads
   ~20 GB of negatives and noise (ACAV100M features, MIT room impulse
   responses, part of AudioSet, FMA) and needs 20,000+ clips for a good model:
   hours of full CPU load on the Mac. Options: a Linux container on the Mac,
   or the notebook on Google Colab (free GPU, under an hour). Output:
   `<output_dir>/<model_name>.onnx`, copied to `/opt/rainbow/voice` via the
   repo and set as `wake_model` / `stop_model`.
   **"Stop" without the wake word** needs a trained "stop" model too (another
   openWakeWord model on the same features, so it costs little); it is
   listened for even while the robot talks, and a false "stop" only stops the
   robot. Until it exists, "stop" works after the wake word, as a local
   command.
2. **Recording**: after the wake word, webrtcvad cuts the utterance (end
   after 0.8 s of silence, 10 s maximum; 4 s without speech gives up). This
   replaces v0-legacy's fixed loudness threshold, which breaks with motor noise.
3. **Local commands**: Vosk (small English model) with the brain's phrase list
   (`/voice/grammar`) runs on each finished utterance (a few ms) and returns the
   matched phrase and its confidence. Anything else maps to "[unk]".
4. **Follow-up window**: for 8 s after the robot finishes speaking, speech is
   recorded without the wake word, so a conversation flows.
5. **Echo**: while espeak is speaking (`/speech/speaking`, latched, from
   `speech_node`) only the stop word is listened for, since the mic hears the
   speaker. The Lite cancels echo on the board; check what its two channels
   carry before relying on it.

The node publishes each utterance (`rainbow_interfaces/Utterance`: 16 kHz
samples, the Vosk phrase and confidence, follow-up or not) to the brain, and
`/voice/stop`. The audio is not compressed: a 10 s clip is 320 kB.

Speech output stays espeak (`speech_node`), for every reply, online or offline.

Eyes (`/eyes`): `open` on the wake word (and while the agent thinks), `happy`
when the robot has finished speaking or nobody spoke.

## Local brain (`rainbow_brain`, new package)

One node, with these parts.

### Modes

| Mode | When | Behaviour |
|---|---|---|
| REMOTE | Mac connected (heartbeat within 3 s) | Runs the agent's skill calls; routes utterances to it |
| LOCAL | Mac not connected | Local commands only; says "I'm offline, I can only do simple things" for anything else |

A third mode that refuses motion (battery cutoff) comes with the battery work,
which is not part of this design yet.

Losing the link in the middle of a goal does not cancel it; the brain just takes
no new decisions from the agent. When the link comes back, the brain sends the
agent a summary of what happened offline.

### Router

For each utterance (`on_utterance` in the brain):

- If Vosk matched a **local command** with confidence 0.8 or more
  (`command_confidence`), the brain runs it immediately, says a short
  confirmation ("Going to the kitchen."), and tells the agent (an `utterance`
  with `handled`), so the conversation and memory stay consistent. The agent
  does not answer it again.
- Otherwise, in REMOTE the audio goes to the agent (base64 PCM in an
  `utterance` message); in LOCAL the robot says it can only go to places, come
  back or wait (not for follow-ups, which are often just noise).

`/voice/stop` stops at once and tells the agent (`stopped_by_voice`).

### Local commands

A fixed phrase list for Vosk (`rainbow_brain/commands.py`), published by the
brain on `/voice/grammar` whenever the places change:

- "stop", "wait"
- "go to the <place>", "go to <place>" (place names from the places file;
  words Vosk does not know cannot be recognised and go to the agent)
- "come back", "go home", "come home" (the place named "home")

One unknown word after a phrase is accepted ("go to the sofa please"); more
means a longer sentence, which goes to the agent. On test clips nothing that
was not a command matched.

### Skills

Implemented once in the brain, used by both the agent and the local commands.
Each call is checked (known place, mode) and may be refused with a reason the
agent can say aloud. If Nav2 cannot reach a place, the brain does not retry: it
reports the failure to the agent (in LOCAL mode it says so itself).

Head skills (`look`) come later, with the head controller (see [Head](#head)).

| Skill | Does |
|---|---|
| `navigate_to(place)` | Nav2 `NavigateToPose` to a named place; reports progress and the result |
| `cancel()` / `stop()` | Cancel the goal / stop now |
| `where_am_i()` | Pose in words: room, nearest place within ~1.5 m, heading, AMCL certainty |
| `get_state()` | Pose, AMCL covariance, battery voltage, current goal, mode |
| `list_places()` | Places with room, description and status |
| `save_place(name, description, room, guess)` | Save the current pose (or the last Foxglove pick); `guess` saves it unconfirmed |
| `edit_place(name, new_name, description, room, confirm)`, `delete_place` | Rename, re-describe or confirm a place; delete it |
| `capture_photo()` | A still JPEG from the RGB camera, with the pose it was taken at |
| `turn(angle)`, `drive(distance)` | Corner cases: Nav2 Spin / DriveOnHeading (forward only) |
| `say(text)` | espeak |
| `set_expression(name)` | Eyes |
| `set_pose_estimate(x, y, yaw, cov)` | Send AMCL an initial pose |

## Places

A place is a named pose on a specific map. Places let the agent reason about
"the kitchen" instead of coordinates. The agent does not see the occupancy grid:
language models are poor at reading distances off pixel grids.

File: `/data/rainbow/places/<map>.yaml` in the container, on a Docker named
volume (`rainbow_data`) declared in `docker-compose.yaml`. The image stays an
appliance: the volume holds only data the robot writes (places, the link
token), never code; `docker compose up` creates it, it survives image rebuilds,
and a fresh install starts with no places and a new token. `maps/` stays
read-only in the image. Places belong
to one map: a new map needs new places.

```yaml
map: home_live
places:                  # a place named "home" is where "go home" goes
  kitchen:
    pose: {x: 3.20, y: -1.10, yaw: 1.57}
    room: kitchen
    description: in front of the fridge, facing the window
    source: owner            # owner | robot | foxglove
    confirmed: true
  sofa:
    pose: {x: 0.80, y: 2.40, yaw: 3.14}
    room: living room
    description: by the grey sofa, facing the TV
    source: robot
    confirmed: false         # the robot's guess, not checked yet
```

Places are added three ways:

1. **By voice**: "Rainbow, remember this spot as the kitchen" saves the current
   pose, confirmed.
2. **By the robot**: the agent sees in a photo that this looks like a kitchen
   and saves it unconfirmed. It says "I think the kitchen is..." about such
   places and may ask the owner to confirm.
3. **In Foxglove**: the 3D panel's publish-pose tool, set to `/places/pick`,
   marks a pose; then name it by voice ("call that the hallway") or with the
   `/places/save` service.

The brain publishes `/places/markers` (`visualization_msgs/MarkerArray`, frame
`map`): an arrow and a text label per place, coloured by confirmed / unconfirmed.
The Foxglove layout gets this layer.

## Link (inside `rainbow_brain`)

A WebSocket server in the brain process (asyncio in its own thread), so the
brain knows the moment the Mac disconnects. The Mac connects to the Pi.
Pi side: `rainbow_brain/link.py` (no ROS, tested on the Mac); Mac side:
`agent/rainbow_agent/link.py` (`LinkRobot`, same interface as the simulated
robot); `agent/tests/test_link.py` runs both ends against each other.

- Port 8766 (8765 is Foxglove). The Mac connects to `rainbow.local`, falling
  back to `192.168.1.172` (configurable).
- One agent at a time: a new connection replaces the old one.
- LAN only. The brain creates a random token on first start
  (`/data/rainbow/link_token`, owner-only); `rainbow-agent pair` copies it to
  the Mac over SSH. A wrong token is refused. Anything with the token can
  drive the robot.
- The agent pings every 1 s; 3 s without a message means disconnected and the
  brain goes back to LOCAL mode.
- The link speaks the agent's units (degrees, metres); the brain converts.
  Results of turn and drive_forward include `now`, where_am_i's sentence, so
  the model reports facts.
- JSON text frames (utterance audio and photos will be binary frames,
  referenced by id).

Messages (built: hello to state; the rest comes with later steps):

| Direction | Type | Content |
|---|---|---|
| Mac → Pi | `hello` | the token; answered by `welcome` (robot, map, protocol) or `error` |
| Mac → Pi | `ping` | heartbeat; answered by `pong` |
| Mac → Pi | `call` | a skill call with id and arguments |
| Pi → Mac | `result` | result of a call, same id |
| Mac → Pi | `say` | text for espeak |
| Pi → Mac | `event` | arrived / failed / cancelled; link_restored (with the events missed while away, last 10) |
| Pi → Mac | `state` | activity, battery, places, mode; on changes and every 5 s |
| Pi → Mac | `utterance` | (step 3) audio, Vosk result, whether it was handled locally |
| Mac → Pi | `status` | what the agent is doing (phase, skill, last heard/reply, timings, tokens), on change; the brain publishes it inside `/brain/status` |
| Mac → Pi | `log` | an agent log line with its level; the brain publishes it on `/agent/log` |
| Pi → Mac | `question` | (later) a decision the brain wants, with a timeout and its local default |
| Mac → Pi | `answer` | (later) answer to a `question` |

In REMOTE mode the brain no longer says failures itself; the agent decides.

For Foxglove the brain publishes `/brain/status` (`rainbow_interfaces/BrainStatus`)
every second and on every change: mode, activity, localization, battery, health
(seconds since the ESP's battery, lidar and odometry messages; Nav2, speech and
face recognition up) and the agent's status as `agent` (last known values, with
`connected` false, while no agent is linked). The layout's Status tab shows it.

Events are rate-limited, and the agent decides whether an event is worth saying
anything about, so the robot does not chatter.

## Agent on the Mac (`rainbow-agent`)

Plain Python, no ROS. The code lives in this repo under `agent/` (the link
protocol is then defined in one place); see `agent/README.md` to install and
run it.

Not in Docker: containers on macOS get no GPU (Docker Desktop's Linux VM has
no Metal), so the model and Whisper would run on the CPU, several times
slower. Docker Model Runner runs models on the host GPU but has no speech to
text. Instead, everything the agent installs lives in one folder,
`~/.rainbow-agent` (llama.cpp's server, the model files, Python and the
dependencies from `uv.lock`, uv's cache, memory, logs), plus one launchd file
once the agent runs as a service; `agent/uninstall.sh` removes both. Every
download is pinned and checked against its SHA-256 (`agent/install.sh`).

### Processes

```
 launchd (once the link exists)
 ├─ rainbow-agent      (one asyncio process, ours)
 │   ├─ link client       WebSocket to the Pi, reconnects, heartbeat
 │   ├─ stt               Whisper, in process
 │   ├─ conversation      session state + tool loop
 │   ├─ events policy     which brain events/questions reach the model
 │   ├─ memory            SQLite + vectors
 │   └─ places matcher    image embeddings for localization (step 6)
 ├─ llama-server :8180  main model: chat + tools + vision (OpenAI-compatible, localhost)
 └─ llama-server :8181  embedding model, for memory search
```

The models run in llama.cpp's own server (a prebuilt macOS binary with Metal,
pinned release), reached through the OpenAI-compatible chat API with `tools`. Swapping models is then a config change, and a model crash
does not take down the link. Speech to text and image embeddings run inside the
agent (small, and called on every utterance).

No agent framework (LangGraph and similar): the loop is small and a framework
adds more surface than it saves. The tool schemas are generated from the skill
table, shared with the brain.

### Conversation loop

1. An `utterance` arrives (or the brain handled it locally: then only the
   transcript and what was done are added to the session).
2. Whisper transcribes it. Empty or nonsense transcripts are dropped silently.
3. The model gets: system prompt (who Rainbow is, short speakable replies, no
   markdown; see [Persona](#persona)), robot state, place list, relevant
   memories, the session so far, and the new utterance.
4. Tool loop: the model calls skills; each result goes back to it. At most ~6
   calls per turn. Long skills (`navigate_to`) return "started" at once; their
   end arrives later as an event, so the conversation is not blocked while the
   robot drives.
5. The final text reply goes to the Pi as `say`.

Events from the brain go through the events policy first (rate limit, drop
duplicates); only some start a model turn (a `question`, a failed goal, a new
face). A `question` gets a turn with a deadline; if the model is late, the
brain's default stands.

### Models

Chosen for an M4 Max MacBook; all run locally, nothing goes to the cloud.
The choice must be confirmed by tests with this robot's real tool calls (below).

| Role | First choice | Alternative | Notes |
|---|---|---|---|
| Speech to text | Whisper large-v3-turbo (mlx-whisper) | Parakeet TDT (parakeet-mlx) | Whisper is the more accurate and multilingual; ~1 s per short utterance on an M4. Parakeet is several times faster but was clearly less accurate in an independent test; worth a try only if latency hurts |
| Main model (chat, tools, vision) | Qwen3-VL 30B-A3B (MoE, ~3B active), Q4_K_M GGUF + F16 vision projector | Same at 8-bit, or Qwen3-VL 32B dense (better, slower) | One model for conversation, tool calls and photos keeps memory use and latency down. The MoE answers fast because few weights are active per token |
| Text embeddings (memory) | Qwen3-Embedding 0.6B (Q8_0 GGUF) | nomic-embed-text | Small; its own llama-server |
| Image embeddings (places) | DINOv2 (ViT-S/B) | SigLIP 2 | DINOv2 is good at "same place" matching; SigLIP adds text search ("photo of a kitchen") |

The Mac has 64 GB, of which macOS lets the GPU use ~48 GB by default; it is
also the owner's everyday laptop, so the agent should stay well under that.
Memory: main model at 4-bit ~18-20 GB, the rest ~2 GB together, so ~22 GB in
all. The selection test also tries the same model at 8-bit (~32 GB) and
Qwen3-VL 32B dense at 4-bit (~20 GB, slower); move up only if the test shows a
clear gain in tool calls or photo answers.

Model selection test, run before step 4 is finished: ~40 recorded or typed
requests with the skill call each should produce ("go to the kitchen", "what's
in front of you?", "remember this spot as the hallway", chit-chat that should
call nothing), plus a few photos. Score correct tool calls, time to first
reply and memory use, per candidate. Keep the set in `agent/tests/` and rerun it
whenever the model or the prompt changes.

Latency budget from the end of speech to the robot starting to talk:
VAD end ~0.8 s + upload ~0.1 s + Whisper ~1 s + model ~1-2 s ≈ 3-4 s. The
eyes' "thinking" expression covers the wait.

### Persona

Rainbow talks with children in the house: playful, warm and short, and limited
in what it says (child-appropriate topics and language, no personal data about
the family to anyone, no instructions for anything unsafe). A basic version in
the system prompt for now; refine it later.

### Running it

- `rainbow-agent run`: the agent for real. Utterances from the robot go
  through Whisper (silence hallucinations like "Thank you." are dropped), the
  conversation, and back as `say`. Commands the robot handled itself are added
  to the session without a reply.
- `rainbow-agent chat [--robot]`: typing instead of talking, against the
  simulated robot or the real one.

### Memory and sessions

Stored on the Mac only (`~/.rainbow-agent/data/memory.db`, SQLite +
embeddings from Qwen3-Embedding on the second llama-server); the Pi never
needs it. Code: `agent/rainbow_agent/memory.py`.

- **Session**: starts at a wake word, ends after 2 min without a turn (checked
  every 30 s) or when the agent stops. The model keeps the full turns of the
  current session.
- **At session end** (in the background; the next reply never waits): the
  model writes a 1-3 sentence summary, stored with an embedding, and lists
  lasting facts (names, preferences, places, things someone asked it to
  remember). Sessions where nobody spoke are not stored.
- **Facts must have been said by a person.** The model turns its own guesses
  into "facts" (it invented a cat's habits and stored them) and the prompt
  alone does not stop it, so a fact is kept only if it has 3+ words and most of
  its meaningful words were said by a person in that session. A fact nearly
  the same as a known one (cosine 0.9) replaces it.
- **At session start**: the first words are embedded; the closest facts and
  episodes (cosine 0.35 or more), plus the last episode, go into the system
  prompt as "things you remember", with the rule never to tell them to
  strangers.
- Memory failures (a server down) are logged and never break a conversation.

Tested with the real models: a new session remembered a child's name,
favourite dinosaur and cat; nothing the robot made up was stored.
Forgetting ("clean memory") is not built yet.

## Camera and faces

The RGB camera is captured by the Hailo container
([hailo_tappas_ros2](https://github.com/kyrikakis/hailo_tappas_ros2)); its face
recognition node already publishes all the brain needs, so that repo does not
change. It must be re-enabled there (it is off to save CPU; measure the Pi's
load with it on).

- **Photos**: `capture_photo` subscribes to
  `/hailo_face_recognition/image_bbx/compressed` (1280x720 JPEG, with the
  detection boxes drawn) only while taking one frame (3 s timeout), and shrinks
  it with OpenCV to 640 wide (~16 kB) before sending it to the agent. The
  agent passes it to the model as an image in the conversation.
- **Faces**: the node publishes `/hailo_face_recognition/detections`
  (`vision_msgs/Detection2DArray`) with the name in the class id
  (`"<label>: <name>"`). The brain sends the agent a `face_seen` event, at most
  once per person per 10 min (`face_repeat`). The agent notes it (speaking
  first is for later).
- **Remembering faces**: `remember_face(name)` calls
  `/hailo_face_recognition/save_face` (`hailo_msgs/srv/SaveGalleryItem`).

Tested on the Pi on domain 77 with fake Hailo topics: photo in 0.1-0.5 s,
no subscription left behind, one face event per person, the pose estimate.

## Startup localization from a photo

AMCL starts with global localization and usually settles once the robot drives
a little. A photo only gives it a hint; it is never treated as a precise pose.

1. **Place photos**: whenever a place is saved through the agent, the robot
   also takes a photo there, kept on the Mac
   (`~/.rainbow-agent/data/place_photos/<map>/`), following renames and
   deletes. Places saved from Foxglove have no photo.
2. **At startup** (once the link is up): the agent calls `get_state()`; if
   AMCL is `sure` or `roughly` localized, it stops there.
3. Otherwise it takes a photo and asks the vision model which place photo
   shows the same spot (current photo first, then up to 6 labelled
   references, each label right before its photo; with the label after the
   photo the model matched the wrong one with 0.98 confidence).
4. A match with confidence 0.6 or more becomes `set_pose_estimate(place)`:
   the brain publishes `/initialpose` (0.5 m, 0.5 rad standard deviation) and
   AMCL refines it from the lidar.

Code: `agent/rainbow_agent/localize.py`. Checked with stand-in photos (three
different images, each found correctly, ~2.5 s); not yet with real rooms.
Once the head controller exists, this can use several head directions.

## Head

The servos have no position feedback, so after a move the head must wait until
it has physically arrived before a photo is taken. A head movement controller
(smooth moves, a settle time that depends on the size of the move) is planned
but not designed yet; it needs testing with the owner watching. Until then,
the brain does not move the head: photos are taken with it centred, and the
`look` skill comes with the controller.

## Build order

1. Places file and data volume, skills, `rainbow_brain` in LOCAL mode, place
   markers in Foxglove.
2. The link in the brain, and the agent's text mode sending raw skill calls.
3. `voice_node`: ReSpeaker, openWakeWord "Rainbow" and "stop", VAD, Vosk,
   router.
4. The agent: Whisper, model with tools, text replies; model selection test.
5. Memory and rolling sessions.
6. Hailo stills and faces (other repo), place photo database, startup
   localization.
7. Head movement controller, then photos in several directions.

Each step is discussed with the owner before it is built.

## Later (agreed, not designed yet)

- Autonomy: the robot exploring on its own to find and name places.
- Speaking first: greeting recognised faces, reporting failures unprompted.
- Forgetting: a "clean memory" command to delete memories.
- Keeping the Mac awake for the agent (for now, Mac asleep = LOCAL mode).
- Recordings on the robot to improve the wake word models.

## Open questions

- Battery: warning and cutoff are out of scope for now; when they are added, the
  brain gets a mode that refuses motion and the agent a battery event.
