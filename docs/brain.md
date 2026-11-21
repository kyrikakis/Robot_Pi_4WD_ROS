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
and speaker (espeak plays on `plughw:2,0`, as before and in v0-legacy). Select
the card by name, not number, which can change between boots. The Lite does
echo cancellation and noise suppression on the board; check what its USB
channels carry before relying on it. English only for now.

One node, `voice_node` (replacing the `wake_word_node` placeholder), opens the
microphone. Capture devices are single-owner, so nothing else may open it.

1. **Wake word**: openWakeWord with a custom-trained "Rainbow" model. A
   one-word wake word triggers falsely more often than a phrase; tune its
   threshold on recordings made with the motors running. Trained on
   synthetic voices for now; recordings on the robot can improve it later.
   **"Stop" needs no wake word**: a second custom openWakeWord model for "stop"
   runs on the same audio (openWakeWord shares the feature extraction between
   models, so it costs little). It stays active even while espeak is speaking
   and during the follow-up window. A false "stop" only stops the robot, which
   is the safe direction, so its threshold can be lower than the wake word's.
2. **Recording**: after the wake word, a VAD (Silero or webrtcvad) cuts the
   utterance (end after ~0.8 s of silence, 10 s maximum). This replaces
   v0-legacy's fixed loudness threshold, which breaks with motor noise.
3. **Local transcript**: Vosk with a fixed phrase grammar (see
   [Local commands](#local-commands)) runs on every utterance and returns the
   matched phrase and its confidence.
4. **Follow-up window**: for ~8 s after the robot finishes speaking, speech is
   recorded without the wake word, so a conversation flows.
5. **Echo**: wake word and recording are muted while espeak is speaking (same
   card, the mic hears the speaker), unless the Lite's echo cancellation
   proves good enough. `speech_node` publishes when it starts and
   stops talking.

The node publishes each utterance (16 kHz mono PCM, the Vosk result, a
timestamp) to the brain. The audio is not compressed: a 10 s clip is 320 kB,
fine on the LAN.

Speech output stays espeak (`speech_node`), for every reply, online or offline.

Eyes show the state: `open` on wake, a "thinking" expression while waiting for
the Mac, `happy` when done (the expression set may need one new entry).

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

For each utterance:

- If Vosk matched a **local command** with high confidence, the brain runs it
  immediately and tells the agent ("user said 'stop', I stopped"), so the
  conversation and memory stay consistent.
- Otherwise, in REMOTE the audio goes to the agent; in LOCAL the robot says
  it is offline.

"Stop" always runs locally and never waits for the Mac; it comes from its own
detector, not through the router (see [Voice](#voice-rainbow_voice)).

### Local commands

A fixed phrase grammar for Vosk, kept small so it stays accurate:

- "wait" (stop is separate, see above)
- "go to the <place>" (place names come from the places file)
- "come back" / "go home" (the place named "home")

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

- The Mac connects to `rainbow.local`, falling back to `192.168.1.172`
  (configurable).
- LAN only, shared token in the first message (stored on the data volume, not
  in the repo; copied to the Mac once). Anything that can reach this socket
  can drive the robot.
- Heartbeat both ways every 1 s; 3 s without one means disconnected.
- JSON text frames; utterance audio and photos as binary frames referenced by
  id.

Messages:

| Direction | Type | Content |
|---|---|---|
| Pi → Mac | `utterance` | audio id, Vosk result, whether it was handled locally |
| Pi → Mac | `event` | goal reached / failed / stuck, stopped by voice, startup, face seen, link restored (with offline summary) |
| Pi → Mac | `question` | a decision the brain wants (e.g. "goal failed twice, what now?"), with a timeout and the local default it will take |
| Pi → Mac | `result` | result of a tool call |
| Mac → Pi | `call` | a skill call with id and arguments |
| Mac → Pi | `answer` | answer to a `question` |
| both | `ping` | heartbeat |

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

### Text mode

Typing instead of talking, for testing and debugging. Before the model is
wired in, the same mode sends raw skill calls (the "stub" of build step 2).

### Memory and sessions

Stored on the Mac only (SQLite + embeddings); the Pi never needs it.

- **Session**: starts at a wake word, ends after ~2 min without speech. The
  model keeps the full turns of the current session.
- **At session end**: the model writes a short summary, stored with an
  embedding, and extracts lasting facts (people, preferences, places, things
  the owner asked it to remember).
- **At session start**: the context gets the relevant facts and summaries
  (embedding search on the first utterance), the robot's state and the place
  list.

## Camera and faces

The RGB camera is captured by the Hailo container
([hailo_tappas_ros2](https://github.com/kyrikakis/hailo_tappas_ros2)). Changes
needed there, not in this repo:

- **Stills on request**: a service returning one JPEG (or a low-rate
  `CompressedImage` topic), for `capture_photo`.
- **Faces**: re-enable face recognition and publish recognised names (with
  `hailo_msgs`). The brain forwards them as events, so the agent can greet
  people and remember who it has seen.

Face recognition was disabled to save CPU; measure the Pi's load with it back
on.

## Startup localization from a photo

AMCL starts with global localization and usually settles once the robot drives
a little. A photo only gives it a hint; it is never treated as a precise pose.

1. At startup the brain sends a `startup` event. The agent calls `get_state()`;
   if AMCL's covariance is already small, it stops there.
2. Otherwise it takes a photo facing forward (head centred).
3. It matches the photo against a **place photo database**: reference photos
   with the poses they were taken at, recorded by driving around the mapped
   area (stored with the places on the data volume). Image embeddings
   (CLIP or DINOv2 style, on the Mac) give candidates; the VLM confirms one.
4. It calls `set_pose_estimate` with that pose and a generous covariance; AMCL
   refines it from the lidar.

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
