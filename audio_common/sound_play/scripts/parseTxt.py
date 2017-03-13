


def parse():
	joke=[]
	##open the file with read only permit
	f = open('/home/turtlebot/turtlebot_ws/src/hri-w17-turtlebot/audio_common/sound_play/scripts/jokes.txt')
	## Read the first line 
	line = f.readline()

	## If the file is not empty keep reading lie one at a time
	## till the file is empty
	while line:
		joke.append(line)
		line = f.readline()
	f.close()
	return joke

