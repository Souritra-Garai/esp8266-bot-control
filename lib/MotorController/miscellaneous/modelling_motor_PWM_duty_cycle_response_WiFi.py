import numpy as np
import socket
import datetime

from numpy.lib.twodim_base import triu_indices_from

s = socket.socket()         
 
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(('0.0.0.0', 3000 ))
s.listen(0)

time=datetime.datetime.now().time()

print('Socket server started listening')

data = []

try :
	
	client, addr = s.accept()

except :

	print('\n')
	s.close()
	exit(1)

try :
    
	while True:

		content = client.recv(1024).decode('ascii')

		if len(content) > 0 : 
			time=datetime.datetime.now().time()
			content=str(time)+','+content
			print(content)
			data.append(content.split(','))

except KeyboardInterrupt:

	print("\nClosing connection")
	client.close()
	s.close()

data = np.array(data, dtype=float)
np.savetxt('Motor PWM Duty Cycle Response.csv', data, delimiter=',')