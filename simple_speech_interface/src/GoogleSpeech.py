import os
import urllib2
import json
import sys
import threading
import subprocess


class audioRecorder(object):

    def __init__(self, fileName, timeout):
        """Constructor for the audioRecorder class"""

        self.fileName = fileName
        self.process = None
        self.timeout = timeout

    def recordFlac(self, fileName):
        """This function actually recors the .flac file calling the sox
        utility"""

        args = ["sox", "-t", "alsa", "default", fileName, "silence", "0", "1",
                "2.5", "2%", "rate", "44.1k"]
        self.process = subprocess.Popen(args)
        self.process.communicate()

    def record(self):
        """This function handles the thread recordi the audio and its
        timeout"""

        thread = threading.Thread(target=self.recordFlac, args=(self.fileName,))
        thread.start()
        thread.join(self.timeout)

        if thread.isAlive():
            print "\nWARNING: recording the audio for speech recognition has"\
                  " timed out"
            self.process.terminate()

        # os.system("play " + self.fileName)
        # uncomment this line to play the audio registered before it's set to
        # google to be recognized. Useful to set the silence threshold of the
        # microphone


class GoogleSpeech():

    def __init__(self, audiofile="./test.flac"):
        """Constructor for the GoogleSpeech class"""
        self.fOrg = audiofile

    def record(self):
        """This function creates a recorder object that handles the threading
        and records the audio input"""

        recorder = audioRecorder(self.fOrg, 10)
        recorder.record()

    def recognize(self):
        """This function builds and sends the request to Google Speech API and
        then print the results"""

        recog = []
        try:
            # build request
            req = urllib2.Request(url="https://www.google.com/speech-api/v2/recognize?output=json&lang=en-us&key=AIzaSyAsj-CMekUogm_fpX1tMXgnWkdWLVjqE7M")
            req.add_header("Content-type", "audio/x-flac; rate=44100")
            req.add_header("Content-length", os.path.getsize(self.fOrg))
            audio = open(self.fOrg, "rb")
            # filesize = os.path.getsize(self.fOrg)
            data = audio.read()
            audio.close()
            req.add_data(data)

            # send request
            response = urllib2.urlopen(req)

            # receive data
            resp = response.read()

            # check data and print
            l = resp.split('\n')
            if len(l) > 2:
                result = []
                recog = json.loads(resp.split('\n')[1])
                if recog["result"].__len__() > 0:
                    for h in recog['result'][0]['alternative']:
                        result.append(h["transcript"])
                        print "utterance: " + h["transcript"]
                    print "\n"
                    return result
            else:
                print "utterance: []"
                return []

        except Exception, e:
            print "No connection to the web. ", e

    def recognizeSpeech(self):
        """A single function to call when we want to record and recognize
        audio"""
        self.record()
        self.recognize()

if __name__ == '__main__':
    if len(sys.argv) > 1:
        sys.stdout = open(sys.argv[1], 'a')

    G = GoogleSpeech("pippo.flac")
    G.recognizeSpeech()
