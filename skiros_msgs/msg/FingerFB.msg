#Finger feedback. It is thought to be valid for every kind of end-effectors fingers, but it has been first drawn for Rq3 

#State can be: 0- in motion, 1- stopped due to contact, 2- at requested position
int16[2] state

#Position
int16[2] pos
	
#Applied force. Valid only when stopped (with Rq3 hopefully could be roughly calculated with the current)
float32[2] appliedFrc