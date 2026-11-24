# What the Mac agent is doing (the agent field of BrainStatus, /brain/status).
# Sent by the agent over the link when something changes; stamp: when it arrived.
builtin_interfaces/Time stamp
# The agent is linked (false: the brain is in local mode; the rest is the last known)
bool connected
# idle, transcribing, thinking, calling_skill, localizing
string phase
# The skill being called, or empty
string skill
# The last thing Whisper heard (after the noise filter), and the last reply spoken
string last_heard
string last_reply
# The last skill call and its result, e.g. "turn(degrees=90) -> ok"
string last_skill
# The last utterance's transcription time, and the last turn's model time (tool calls
# included) (s)
float32 stt_seconds
float32 turn_seconds
# The last turn: tool calls, and the tokens the model read and wrote (all its calls)
uint32 tool_calls
uint32 prompt_tokens
uint32 completion_tokens
# This session: turns so far, its age (s), memories recalled at its start
uint32 session_turns
float32 session_seconds
uint32 memories
# The chat model, and how long the agent has been running (s)
string model
float32 uptime
