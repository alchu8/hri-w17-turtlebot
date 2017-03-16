import rospy
import random
import parseTxt
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

jokes = []

def speak_func():
	# Ordered this way to minimize wait time.
	soundhandle = SoundClient()
	rospy.sleep(1)
	#soundhandle.say('Take me to your leader.','')
	voice = 'voice_kal_diphone'
	volume = 100.0
	jokes = parseTxt.parse()
	index = random.randint(0,len(jokes)-1)
	s = jokes[index]
	print 'Saying: %s' % s
	print 'Voice: %s' % voice

	soundhandle.say(s, voice)
	rospy.sleep(1)
	return len(s.split())

def welcome():
	soundhandle1 = SoundClient()
	rospy.sleep(1)
	voice = 'voice_kal_diphone'
	volume = 100.0
	s = "Do you want to hear a joke?"
	soundhandle1.say(s,voice)
	rospy.sleep(1)
	
def speak_whatever(s):
	soundhandle2 = SoundClient()
	rospy.sleep(1)
	voice = 'voice_kal_diphone'
	volume = 100.0
	soundhandle2.say(s,voice)
	rospy.sleep(1)

def play_wav(s):
	soundhandle3 = SoundClient()
	rospy.sleep(1)
	volume = 100.0
	soundhandle3.playWave(s,volume)
	rospy.sleep(1)
