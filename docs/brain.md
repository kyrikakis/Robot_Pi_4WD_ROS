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
   **Wake words**: trained for Rainbow on the Mac (`training/wakeword/`, openWakeWord's
   training on synthetic voices only: LibriTTS speakers, accented Piper voices,
   Greek pronunciations), shipped in the package (`rainbow_voice/models/`).
   `wake_models` is a list, any one wakes, each with its threshold in
   `wake_thresholds`. `hey_rainbow.onnx` answers "hey Rainbow" and "Rainbow"
   alone (threshold 0.3; the "hey"-only round is kept in the training results).
   `hey_robot.onnx` ("hey Robot" and "Robot" alone) ships too but is not in
   `wake_models`: on synthetic voices at 0.3 "hey Robot" scored 82% in a quiet
   room (72/62/47% at 10/5/0 dB, 42% across the room, ~0.4 false wakes per hour),
   but the owner's "Robot" through the ReSpeaker stayed under 0.3. On the robot
   the owner finds "hey Rainbow" far better than "hey Jarvis". On the synthetic test
   (standard voices, 0.5): "hey Rainbow" 100% in a quiet room, 84/69/42% at
   10/5/0 dB noise, 45% far across a reverberant room, 0.7 false wakes per hour of
   AudioSet; "hey Jarvis" scores 99, 95/89/71, 78, 0.2: ours still trail in noise
   and at a distance. It also wakes when the robot says "Rainbow" itself, which
   is why nothing but the stop word is listened for while the robot talks.
   **"Stop"** without the wake word: `stop.onnx` (`stop_model`, threshold 0.7),
   listened for even while the robot talks; a false "stop" only stops the robot.
   Round 4 at 0.7: 78% in a quiet room, 60/49/36% at 10/5/0 dB, ~2 false stops
   per hour of AudioSet, ~17 per hour of espeak (a short test); a bare "Stop!" 29
   of 32 synthetic clips. 0.9 was tried first: the owner's "stop" through the
   ReSpeaker scored 0.75-0.90 and was missed. It
   does not fire on "Rainbow, stop" or "Robot, stop": those go through Vosk, as a
   local command after the wake word (17-18 of 20 on the synthetic end-to-end
   test). For that, the recording starts with the chunk the wake word
   fired on, a leading unknown word (the wake word's tail) is dropped by the
   recognizer, and the wake word does not silence the stop word.
2. **Recording**: after the wake word, webrtcvad cuts the utterance (end
   after 0.8 s of silence, 10 s maximum; 4 s without speech gives up). The
   silence only counts once 0.24 s of continuous speech was heard: the tail of
   the wake word or of the hum cut commands off at 1.2 s. The wake hum
   (`wake_beep`) is off by default: its 0.32 s, skipped from the recording since
   the mic hears it, swallowed commands said in one breath ("Rainbow, stop"). This replaces
   v0-legacy's fixed loudness threshold, which breaks with motor noise.
3. **Local commands**: Vosk (small English model) with the brain's phrase list
   (`/voice/grammar`) runs on each finished utterance (a few ms) and returns the
   matched phrase and its confidence. Anything else maps to "[unk]".
4. **Follow-up window**: for 8 s after the robot finishes speaking, speech is
   recorded without the wake word, so a conversation flows (at most two in a
   row). The wake word still works in it: during a follow-up recording it
   restarts as a normal wake. No follow-up starts while the robot moves
   (`/cmd_vel` non-zero, and 0.5 s after): motor noise started them, and
   each one kept the wake word deaf for up to 10 s.
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

**Low battery** comes on top of either mode (`rainbow_brain/battery.py`): at
<= 7.0 V for 30 s (the pack dips while driving, so one reading means little)
the brain cancels every motion, refuses new ones ("my battery is empty"),
centres the head and stops `rainbow_localization`, `rainbow_tof`, `rainbow_slam`
and `rainbow_navigation` with `supervisorctl` (there is no dock yet, so driving
is pointless, and Nav2 alone costs ~40% of a core). The ESP link, base (eyes,
EKF), voice, brain and Foxglove keep running: it can still be talked to and
seen. Every 2 min (`battery_repeat_s`) it says "My battery is empty. Please
charge me." with sad eyes. The agent gets a `battery_low` event (not spoken
again) and sees it in the state. Once >= 7.6 V has held for 1 min (a charged
battery) the stopped programs start again, the map first, and it says thank
you (`battery_ok`).

**Picked up** also comes on top of either mode (`rainbow_brain/lift.py`, from
the BNO055): the robot rising > 5 cm (8 cm while it is commanded to move) or
tilting > 20 deg from its rest counts as lifted; 2 s still counts as put down.
Tuned on a recording (three lifts, a push over a carpet edge, which rose 2.7 cm
by the same estimate and is not a lift). Lifted: every motion is cancelled and
refused ("someone is holding me"), `/lifted` is true, so the lidar goes to
standby, and the agent gets a `lifted` event (not spoken). Put down: AMCL
restarts global localization (the location is lost) and the agent gets
`put_down`; the robot never moves by itself after that, the next move or "look
around" relocalizes. The lidar is frugal (`lidar_power`, see AGENTS.md): every
move asks for it on `/lidar/wanted` and waits for the first scan (<= 8 s) before
Nav2 gets the goal. The detector's IMU, with `/tf`, `/cmd_vel` and `/odom_raw`,
is received on a second node (`brain_inputs`) with its own single-threaded
executor: on the brain node itself, every message cost a wait-set rebuild over all
its entities (~23% of a core). That thread costs ~9% of a core, the whole brain
~11% at rest (2026-11-26). The detector could move onto the ESP with the board
IMU later.

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
- "look around", "find yourself", "localize"
- The head, so it can be controlled without the Mac: "stop watching", "stop
  following" (ends any tracking, centres the head and switches greeting people
  off), "watch for people", "greet people" (switches it on), "watch me" (follows
  the face in front of it)

One unknown word after a phrase is accepted ("go to the sofa please"); more
means a longer sentence, which goes to the agent. On test clips nothing that
was not a command matched.

### Skills

Implemented once in the brain, used by both the agent and the local commands.
Each call is checked (known place, mode) and may be refused with a reason the
agent can say aloud. If Nav2 cannot reach a place, the brain does not retry: it
reports the failure to the agent (in LOCAL mode it says so itself).

The head has `look` (see [Head](#head)).

| Skill | Does |
|---|---|
| `navigate_to(place)` | Nav2 `NavigateToPose` to a named place; reports progress and the result |
| `cancel()` / `stop()` | Cancel the goal / stop now |
| `where_am_i()` | Pose in words: room, nearest place within ~1.5 m, heading, AMCL certainty |
| `get_state()` | Pose, AMCL covariance, battery voltage, current goal, mode |
| `list_places()` | Places with room, description and status |
| `save_place(name, description, room, guess)` | Save the current pose (or the last Foxglove pick); `guess` saves it unconfirmed |
| `edit_place(name, new_name, description, room, confirm)`, `delete_place` | Rename, re-describe or confirm a place; delete it |
| `capture_photo()` | A still JPEG from the RGB camera, with the pose it was taken at; skips frames while the head moves |
| `look(pan_degrees, tilt_degrees)` | Point the head (+ left, + up from straight ahead); returns once it has settled |
| `track(what)`, `stop_tracking()` | Keep the head on a face, a person (by name) or a detected thing; `track_lost` event when it goes out of view; stopping centres the head |
| `greet_people(enabled)` | Turn greeting people who come by on or off (saved; on by default) |
| `remember_face(name)`, `forget_face(name)` | Save the face in view under a name (after the person confirmed it), or delete one |
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
  once per person per 10 min (`face_repeat`). The agent notes it. A face it
  does not know (only the face boxes count, not COCO's), in view for 3 s
  (`unknown_after`), becomes a `face_unknown` event, at most once per 10 min;
  the agent answers it at once: it greets them and asks their name.
- **Remembering faces**: `remember_face(name)` calls
  `/hailo_face_recognition/save_face` (`hailo_msgs/srv/SaveGalleryItem`); an
  all-lowercase name is saved capitalised. The agent says the name back and
  saves only after a yes; a guard in `agent/rainbow_agent/guards.py` enforces it
  (the model saved names at once, and invented "Alex" when none was said). A face
  already saved under another name is refused with that name ("I already know
  this face as Bambu"); `forget_face(name)` (`.../delete_face`) removes one, and
  after it the agent saves the confirmed name. The Hailo node answers 1 for all
  three "already known" cases and tells them apart only in its message
  (`rainbow_brain/faces.py`). A name saved with a face unlike the one in view
  ("I know a face called Stefanos that looks different") makes the agent ask if
  it is the same person (another angle or light); only after a yes does
  `remember_face(name, same_person=true)` add this view to that name (Hailo
  `append`; the agent's guard drops a same_person the person did not confirm).
  `append` with a face close to someone else's renames them, so it is used only
  after that answer. After a new face is saved the brain collects up to two more
  views itself for 12 s while the person turns their head (the agent asks them
  to). Face events sent while the agent was offline are not lost: when it
  connects, and when a look starts, the faces in view are reported again. Answers
  to the agent's questions skip the local phrase list (Vosk heard "Stefanos" as
  "stop"); the stop word still works then.
- **Greeting people** (`rainbow_brain/greeting.py`): someone in view (COCO's
  `person`, e.g. legs, or a face) for 1 s while the robot is idle gets one
  look: the brain has the tracker follow "face" (falling back to the body's top),
  for 20 s, then the head centres. The tracker moves the head at 200 deg/s, calmer
  than the head's 400 deg/s default (full speed looked jumpy when following). Face recognition then names them: a
  `face_seen` event with `greet` true, which the agent answers at once with a
  greeting (in local mode the brain says "Hi <name>!"); an unknown face is
  `face_unknown` (asks the name). One look per presence: they must be out of view
  for 10 s to be noticed again; a person is greeted at most once per 10 min. Not
  while driving (the ToF in the head must look ahead), in low battery, or while
  the head follows something asked for; driving, `look` and `track` take the head
  back. `greet_people(false)` ("stop greeting people") switches it off, also the
  unknown-face question; saved in `<data_dir>/settings.json`. "stop watching"
  only ends the current follow.

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
   AMCL is `sure`, it stops there (`roughly` can be a metre off, so it is not
   taken as localized anywhere).
3. Otherwise it takes a photo and asks the vision model which place photo
   shows the same spot (current photo first, then up to 6 labelled
   references, each label right before its photo; with the label after the
   photo the model matched the wrong one with 0.98 confidence).
4. A match with confidence 0.6 or more becomes `set_pose_estimate(place)`:
   the brain publishes `/initialpose` (0.5 m, 0.5 rad standard deviation) and
   AMCL refines it from the lidar.

Code: `agent/rainbow_agent/localize.py`. Checked with stand-in photos (three
different images, each found correctly, ~2.5 s); not yet with real rooms.
With the head controller (see [Head](#head)) this can use several head directions (next).

## Head

The servos have no position feedback and jump at full speed to whatever angle
the ESP is sent, so the head node (`rainbow_drivers/head_node.py`, its own
supervisor program `rainbow_head`) controls the motion:

- The commanded angle moves towards the target with limited speed and
  acceleration, braking to stop on the target; both axes are scaled to arrive
  together. The default is near the servos' own speed (400 deg/s, 4000
  deg/s^2: a 30 deg move takes ~0.2 s): the ESP takes whole degrees and the
  servos visibly step at each one, so slow glides (5 to 60 deg/s, tried with
  the owner) all looked jerky. A goal can still ask for a slower speed. Code:
  `rainbow_drivers/head_motion.py`, tested without ROS.
- A new target blends in from the current motion, so a stream of targets is one
  continuous movement: `/head/target` (`HeadTarget`, latest wins) is for
  following faces later.
- `/joint_states` follows the commanded angle at 50 Hz while moving (10 Hz at
  rest), `servo_latency_s` late, so the ToF and camera frames are placed with the
  pose the head really had. This matters for the planned ToF + camera fusion
  (the distance of what the camera sees, placed in the map).
- After a move, the head counts as settled only after `settle_base_s` +
  `settle_per_deg_s` x the degrees moved; `/head/moving` is true until then.
- `/head/move` (`MoveHead` action) returns once settled; a new target of any
  kind replaces it, cancelling stops the head where it is. `/head/set_pose`,
  `/head/center` and the old degree topics start a move without waiting.

The brain's `look` uses `/head/move`; `capture_photo` skips frames while
`/head/moving` is true.

**Tracking** (`rainbow_brain/tracker_node.py`, started by the brain's `track`
skill): the tracker follows a target in the detections (any
`vision_msgs/Detection2DArray` topic in `detection_topics`) by sending
`/head/target` with each detection. It aims from the head pose at the time of
the frame (`/joint_states` history), not the current one, so the head's own
motion and the detector's delay do not make it overshoot; `gain` 0.8 damps
noisy boxes, offsets under 1.5 deg are ignored. Targets
(`rainbow_brain/tracking.py`): "face" (anyone; "me", "someone" map to it), a
person's name (face recognition's `face: <name>`), or an object detector's
label ("ball" maps to COCO's "sports ball"). Several matches: the one nearest the
last, or at the start the largest. "face" falls back to a person's body
(COCO's `person`, aimed at the top of the box) when no face is visible. The
head stays still while the target is in a quiet zone in the middle of the view
(8 deg pan, 6 deg tilt off centre). Out of view, the head stays where it is
for 3 s; then the status says lost, the head goes back to the centre and the
agent gets a `track_lost` event. `stop_tracking` and `stop` end tracking and
centre the head; `look` ends it and turns the head where asked. Only the head moves: the robot does not drive after
anything.

The Hailo container runs face recognition and an object detector (COCO's 80
classes: person, sports ball, backpack, cup, ...) in one pipeline, so faces,
known people and those things can be tracked; tracking.py maps everyday words
("ball", "phone") to COCO's names. The camera's FOV (66 x 41 deg, the Camera
Module 3's datasheet) and the frame delay (0.15 s: the Hailo stamps detections
when it publishes them, after the frame was taken) are not measured yet. The
Hailo's boxes are normalized (0-1); a detector sending pixels needs
`normalized_boxes` false. Speed, settle time and latency are estimates until
measured on the robot with the owner watching.

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
8. Recognising voices (later): a pre-trained speaker-embedding model (a few MB,
   e.g. WeSpeaker or 3D-Speaker through sherpa-onnx, pinned) in `rainbow_voice`
   turns each utterance into a voice print and names the closest enrolled
   person, or "unknown", on the utterance. Enrolment by voice ("Rainbow,
   remember my voice"): 5-10 sentences per person through the ReSpeaker, so the
   robot's own noise is in the prints; prints on the data volume. No training.
   Expect: reliable among a few household members from ~2 s of speech, weak on
   a bare "stop", alike voices can mix. Not a security check (a recording fools
   it): for names, per-person memory and cross-checking faces. To discuss: who
   is enrolled and what changes once the speaker is known.

Each step is discussed with the owner before it is built.

## Later (agreed, not designed yet)

- Autonomy: the robot exploring on its own to find and name places.
- Speaking first: greeting recognised faces, reporting failures unprompted.
- Forgetting: a "clean memory" command to delete memories.
- Keeping the Mac awake for the agent (for now, Mac asleep = LOCAL mode).
- Recordings on the robot to improve the wake word models.

## Open questions

