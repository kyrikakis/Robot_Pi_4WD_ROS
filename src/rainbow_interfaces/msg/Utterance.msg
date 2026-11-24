# What someone said to Rainbow (rainbow_voice's voice_node -> rainbow_brain).
builtin_interfaces/Time stamp
# 16 kHz mono 16-bit samples, from the end of the wake word to the end of speech
uint32 sample_rate
int16[] samples
# Recorded in the follow-up window after the robot spoke, without the wake word
bool follow_up
# A local command phrase Vosk matched (from /voice/grammar), or empty, and its
# confidence (the lowest word confidence, 0..1)
string command
float32 command_confidence
