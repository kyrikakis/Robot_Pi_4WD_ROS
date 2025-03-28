import pvporcupine
import pyaudio
import numpy as np
import openai
import wave
import tempfile
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import UInt16
from std_msgs.msg import Int32
from threading import Thread
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class SpeechListenerNode(Node):
    def __init__(self):
        super().__init__('speech_publisher')
        # Initialize Picovoice Porcupine
        self.porcupine = pvporcupine.create(access_key=os.environ['PICOVOICE_API_KEY'], 
                    keyword_paths=["/workspaces/Robot_Pi_4WD_ROS/speech-recognition/smiley_en_raspberry-pi_v3_0_0.ppn",
                                "/workspaces/Robot_Pi_4WD_ROS/speech-recognition/robot_en_raspberry-pi_v3_0_0.ppn"])
                                
        self.audio_stream = pyaudio.PyAudio().open(
            rate=self.porcupine.sample_rate,
            channels=1,
            format=pyaudio.paInt16,
            input=True,
            frames_per_buffer=self.porcupine.frame_length
        )
        self.speech_publisher = self.create_publisher(String, '/speech/speak', 10)
        self.y_publisher = self.create_publisher(Int32, '/head/y_rotation', 10)
        self.z_publisher = self.create_publisher(Int32, '/head/z_rotation', 10)
        self.eyes_publisher = self.create_publisher(String, '/eyes', 
                                                    QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1))
        self.openai_client = openai.OpenAI()

        self.subscription = self.create_subscription(
            UInt16,
            'battery',
            self.listener_callback,
            10)
        self.battery_level = 0

        thread = Thread(target = self.run)
        thread.start()
        
    def listener_callback(self, msg):
        self.battery_level = msg.data

    def record_until_silence(self, wait_seconds=10):
        """ Records audio until silence is detected or a max duration is reached. """
        frames = []
        frames_count = 0
        noise_detected = False
        silence_threshold = 20000  # Adjust based on your environment
        silence_duration = 0
        silence_duration_sec = 1  # Count how long silence lasts
        silence_duration_frames = int(silence_duration_sec * self.porcupine.sample_rate / self.porcupine.frame_length)
        waiting_duration_frames = int(wait_seconds * self.porcupine.sample_rate / self.porcupine.frame_length)

        print("Recording")

        while True:
            pcm = self.audio_stream.read(self.porcupine.frame_length, exception_on_overflow=False)
            pcm_array = np.frombuffer(pcm, dtype=np.int16)
            frames.append(pcm)
            frames_count +=1

            # Calculate the amplitude of the current frame to detect silence
            amplitude = np.max(np.abs(pcm_array))
            print("aplitute", amplitude)
            if amplitude < silence_threshold:
                silence_duration += 1  # Increase silence counter if current frame is silent
            else:
                silence_duration = 0  # Reset silence counter if there's noise
                noise_detected = True

            # If silence lasts for a certain number of frames, stop recording
            if noise_detected:
                if silence_duration > silence_duration_frames:  # Adjust this to control how long silence is detected before stopping
                    print("Silence detected.")
                    break
            else:
                if frames_count > waiting_duration_frames:
                    print("Nothing detected, stopping.")
                    break

        print("Recording complete.")
        return b''.join(frames)

    def transcribe_audio(self, audio_data):
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
                response = self.openai_client.audio.transcriptions.create(model="whisper-1", file=audio_file, language="en")
                # Clean up temporary file
                os.remove(tmp_wav_filename)
                return response.text
        except openai.BadRequestError as e:
            print(e)

    def speak(self, text, voice="mb-us2", speed=120, gap=0.9, device="plughw:2,0", amplitude=40):
        msg = String()
        msg.data = text
        self.speech_publisher.publish(msg)

    def run(self):
        try:
            print("Listening for wake word...")
            while True:
                pcm = self.audio_stream.read(self.porcupine.frame_length, exception_on_overflow=False)
                pcm_array = np.frombuffer(pcm, dtype=np.int16)
                
                keyword_index = self.porcupine.process(pcm_array)
                if keyword_index >= 0:
                    print("Wake word detected!")
                    msg = String()
                    msg.data = 'open'
                    self.eyes_publisher.publish(msg)
                    audio_data = self.record_until_silence()
                    text = self.transcribe_audio(audio_data)
                    print("Transcription:", text)
                    if(text.find("battery level") > 0):
                        self.speak("My battery level is: " + str(self.battery_level/10) + " volts, I think I can make it")
                    elif(text.find("Go to your room") > -1):
                        self.speak("I don't have a room, we need a bigger place")
                    elif(text.find("How are you") > -1):
                        self.speak("I'm good! Thank you. What's your name?")
                        audio_data = self.record_until_silence(10)
                        text = self.transcribe_audio(audio_data)
                        print("Transcription:", text)
                        if(text.find("My name is") > -1):
                            name =  text[11:]
                            self.speak("Nice to meet you: " + name)
                        else:
                            name =  text[0:]
                            self.speak("Nice to meet you: " + name)
                    elif(text.find("Look right") > -1):
                        z_msg = Int32()
                        z_msg.data = -40
                        self.z_publisher.publish(z_msg)
                    elif(text.find("Look left") > -1):
                        z_msg = Int32()
                        z_msg.data = 38
                        self.z_publisher.publish(z_msg)
                    elif(text.find("Look up") > -1):
                        y_msg = Int32()
                        y_msg.data = -36
                        self.y_publisher.publish(y_msg)
                    elif(text.find("Move your head down") > -1):
                        y_msg = Int32()
                        y_msg.data = 19
                        self.y_publisher.publish(y_msg)
                    elif(text.find("Look straight") > -1):
                        y_msg = Int32()
                        y_msg.data = 0
                        self.y_publisher.publish(y_msg)
                        z_msg = Int32()
                        z_msg.data = 0
                        self.z_publisher.publish(z_msg)
                    else:
                        self.speak(text)
                    
                    msg = String()
                    msg.data = 'happy'
                    self.eyes_publisher.publish(msg)
        finally:
            self.audio_stream.stop_stream()
            self.audio_stream.close()
            self.porcupine.delete()

def main(args=None):
    rclpy.init(args=args)
    speech_listener = SpeechListenerNode()
    rclpy.spin(speech_listener)
    speech_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
