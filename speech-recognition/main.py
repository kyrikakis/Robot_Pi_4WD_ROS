#!/usr/bin/env python3
import speech_recognition as sr
import time

# obtain audio from the microphone
r = sr.Recognizer()
with sr.Microphone() as source:
    while True:
        try:
            print("Say something!")
            audio = r.listen(source, snowboy_configuration=["/snowboy/swig/Python3", ["/workspaces/Robot_Pi_4WD_ROS/speech-recognition/kiro.pmdl",
                                                                                      "/workspaces/Robot_Pi_4WD_ROS/speech-recognition/robot.pmdl"]])
            print("hotword recognized!")
            # recognize speech using whisper
            start = time.time()
            print("Whisper thinks you said:" + r.recognize_openai(audio))
            print("time elapsed in seconds:", time.time() - start)
        except sr.UnknownValueError:
            print("Whisper could not understand audio")
        except sr.RequestError as e:
            print(f"Could not request results from Whisper; {e}")
        except KeyboardInterrupt:
            print(f"Could not request results from Whisper; {e}")