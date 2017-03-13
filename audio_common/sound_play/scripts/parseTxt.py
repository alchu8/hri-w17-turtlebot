


def parse():
	joke=[]
	##open the file with read only permit
	f = open('/home/turtlebot/turtlebot_ws/src/alchu8/audio_common/sound_play/scripts/jokes.txt')
	## Read the first line 
	line = f.readline()

	## If the file is not empty keep reading line one at a time
	## till the file is empty
	while line:
		joke.append(line)
		line = f.readline()
	f.close()
	return joke

