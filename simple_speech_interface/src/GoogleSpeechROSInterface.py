#!/usr/bin/env python

import os
import urllib2
import json
import rospy
import threading
import subprocess
import time

import roslib
roslib.load_manifest("simple_speech_interface")

from simple_speech_interface.msg import SpeechRecognitionMsg


class audioRecorder(object):

    def __init__(self, fileName, timeout):
        """Constructor for the audioRecorder class"""

        self.fileName = fileName
        self.process = None
        self.timeout = timeout

    def recordFlac(self, fileName):
        """This function actually recors the .flac file calling the sox
        utility"""

        args = "sox -t alsa default %s silence 0 1 2.5 2%% rate 44.1k"%fileName
        self.process = subprocess.Popen(args, shell=True)
        self.process.communicate()

    def record(self):
        """This function handles the thread and its timeout"""
        thread = threading.Thread(target=self.recordFlac, args=(self.fileName,))
        thread.start()
        thread.join(self.timeout)
        if thread.isAlive():
            print "\nWARNING: recording the audio for speech recognition has"\
                  " timed out"
            self.process.terminate()

        #os.system("play " + self.fileName)
        # uncomment this line to play the audio registered before it's set to
        # google to be recognized. Useful to set the silence threshold of the
        # microphone


class GoogleSpeechROSInterface():

    def __init__(self, audiofile="./test.flac"):
        """Constructor for the GoogleSpeech class"""
        self.sPublisher = rospy.Publisher('SpeechRecognition/Utterances', SpeechRecognitionMsg, queue_size = 10)
        self.fOrg = audiofile

    def record(self):
        """This function creates a recorder object that handles the threading
        and records the audio input"""
        recorder = audioRecorder(self.fOrg, 10)
        print(time.ctime() + " - Started Recording\n")
        recorder.record()
        print(time.ctime() + " - Stopped Recording\n")

    def recognize(self):
        """This function builds and sends the request to Google Speech API and
        then print the results"""

        recog = []
        try:
            # build request
            req = urllib2.Request(url="https://www.google.com/speech-api/v2/"
                                  "recognize?output=json&lang=en-us&key="
                                  "AIzaSyAsj-CMekUogm_fpX1tMXgnWkdWLVjqE7M")
            req.add_header("Content-type", "audio/x-flac; rate=44100")
            req.add_header("Content-length", os.path.getsize(self.fOrg))
            audio = open(self.fOrg, "rb")
            data = audio.read()
            audio.close()
            req.add_data(data)

            # send request
            print(time.ctime() + " - Request Sent\n")
            response = urllib2.urlopen(req)

            # receive answer
            resp = response.read()

            # chek answer and publish message
            l = resp.split('\n')
            if len(l) > 2:
                recog = json.loads(resp.split('\n')[1])
                if recog["result"].__len__() > 0:
                    speechMsg = SpeechRecognitionMsg()
                    for h in recog['result'][0]['alternative']:
                        speechMsg.utterances.append(h["transcript"].
                                                    encode('utf-8'))
                        if len(h) > 1:
                            speechMsg.confidence = float(h["confidence"])
                    self.sPublisher.publish(speechMsg)
                    print "published message: %s"%speechMsg

            else:
                speechMsg = SpeechRecognitionMsg()
                self.sPublisher.publish(speechMsg)
            print(time.ctime() + " - Request Received\n")

        except Exception, e:
            print e
            print('I am sorry I lost my connection to the web. Please wait one minute while I try to reconnect.')

    def recognizeSpeech(self):
        self.record()
        self.recognize()
        os.remove(self.fOrg)

if __name__ == '__main__':

    rospy.init_node("GoogleSpeechROSInterface")
    G = GoogleSpeechROSInterface("pippo.flac")
    raw_input()
    G.recognizeSpeech()
