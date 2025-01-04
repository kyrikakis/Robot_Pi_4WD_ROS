import pvporcupine
import pyaudio
import numpy as np
import openai
import wave
import tempfile
import os
import subprocess

# Initialize Picovoice Porcupine
porcupine = pvporcupine.create(access_key=os.environ['PICOVOICE_API_KEY'], keywords=["porcupine"])
audio_stream = pyaudio.PyAudio().open(
    rate=porcupine.sample_rate,
    channels=1,
    format=pyaudio.paInt16,
    input=True,
    frames_per_buffer=porcupine.frame_length
)

openai_client = openai.OpenAI()

def record_until_silence():
    """ Records audio until silence is detected or a max duration is reached. """
    frames = []
    silence_threshold = 20000  # Adjust based on your environment
    silence_duration = 0
    silence_duration_sec = 2  # Count how long silence lasts
    silence_duration_frames = int(silence_duration_sec * porcupine.sample_rate / porcupine.frame_length)

    print("Recording")

    while True:
        pcm = audio_stream.read(porcupine.frame_length, exception_on_overflow=False)
        pcm_array = np.frombuffer(pcm, dtype=np.int16)
        frames.append(pcm)

        # Calculate the amplitude of the current frame to detect silence
        amplitude = np.max(np.abs(pcm_array))
        print("aplitute", amplitude)
        if amplitude < silence_threshold:
            silence_duration += 1  # Increase silence counter if current frame is silent
        else:
            silence_duration = 0  # Reset silence counter if there's noise

        # If silence lasts for a certain number of frames, stop recording
        if silence_duration > silence_duration_frames:  # Adjust this to control how long silence is detected before stopping
            print("Silence detected.")
            break

    print("Recording complete.")
    return b''.join(frames)

def transcribe_audio(audio_data):
    """ Converts raw audio to WAV and sends it to Whisper for transcription. """
    with tempfile.NamedTemporaryFile(delete=False, suffix=".wav") as tmpfile:
            tmp_wav_filename = tmpfile.name
            # Write audio data to temporary .wav file
            with wave.open(tmpfile, 'wb') as wf:
                wf.setnchannels(1)  # Mono channel
                wf.setsampwidth(pyaudio.PyAudio().get_sample_size(pyaudio.paInt16))  # 16-bit
                wf.setframerate(16000)  # Whisper expects 16kHz sample rate
                wf.writeframes(audio_data)

    # Open the temp .wav file and send to Whisper
    try:
        with open(tmp_wav_filename, 'rb') as audio_file:
            response = openai_client.audio.transcriptions.create(model="whisper-1", file=audio_file, language="en")
            # Clean up temporary file
            os.remove(tmp_wav_filename)
            return response.text
    except openai.BadRequestError as e:
        print(e)

def speak(text, voice="en-gb", speed=None, device="plughw:2,0", amplitude=50):
    cmd = ["espeak"] + (["-v", voice] if voice else []) + (["-s", str(speed)] if speed else [])
    try:
        cmd += ["-a", str(amplitude), "-w", "temp.wav", text]
        subprocess.run(cmd, check=True, capture_output=True)
        subprocess.run(["aplay", "-D", device, "temp.wav"], check=True, capture_output=True)
        os.remove("temp.wav")
    except (subprocess.CalledProcessError, FileNotFoundError) as e: print(f"Error: {e}")

try:
    print("Listening for wake word...")
    while True:
        pcm = audio_stream.read(porcupine.frame_length, exception_on_overflow=False)
        pcm_array = np.frombuffer(pcm, dtype=np.int16)
        
        keyword_index = porcupine.process(pcm_array)
        if keyword_index >= 0:
            print("Wake word detected!")
            audio_data = record_until_silence()
            text = transcribe_audio(audio_data)
            print(f"Transcribed Text: {text}")
            speak(text)

except KeyboardInterrupt:
    print("Stopping...")
finally:
    audio_stream.stop_stream()
    audio_stream.close()
    porcupine.delete()
