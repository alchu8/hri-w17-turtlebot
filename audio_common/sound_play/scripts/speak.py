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

