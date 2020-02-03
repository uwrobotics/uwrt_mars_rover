import subprocess

s = subprocess.check_output('docker ps', shell=True)
print 'Results of docker ps:\n' + s

# Here is the result after running the required docker command
# CONTAINER ID        IMAGE                   COMMAND                  CREATED             STATUS              PORTS                    NAMES
# 0b3a38d185d3        danielsnider/mapproxy   "/bin/sh -c /start.sh"   43 seconds ago      Up 34 seconds       0.0.0.0:8080->8080/tcp   mystifying_bell

if s.find('mystifying_bell2') != -1:
	print('Required mapproxy container is found.')
else:
	print('Required mapproxy container could not found')
	print('Mapproxy is started.')
	subprocess.call(['docker','run','-p','8080:8080','-d', '-t', '-v', '/home/mapproxy:/mapproxy', 'danielsnider/mapproxy'])


	
