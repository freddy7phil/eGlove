#!/usr/bin/python

from __future__ import absolute_import, print_function, unicode_literals

from optparse import OptionParser, make_option
import os
import sys
import socket
import uuid
import dbus
import dbus.service
import dbus.mainloop.glib
import numpy as np
import time
from sklearn.externals import joblib

try:
  from gi.repository import GObject
except ImportError:
  import gobject as GObject
  
#clf = joblib.load('Trainer/trainer.pkl')
#clf = joblib.load('MLP_Trainer_100/trainer.pkl')
#print("Trainer loaded")
getAlphabet = dict()
getAlphabet = {1.0 : 'A', 2.0 : 'B', 3.0: 'C', 4.0: 'D', 5.0 : 'E', 6.0: 'F',7.0: 'G', 8.0: 'H', 
               9.0 : 'I', 11.0 : 'K', 12.0 : 'L' , 13.0 : 'M', 14.0 : 'N', 15.0 : 'O', 17.0 : 'Q',
                18.0 : 'R', 19.0 : 'S', 20.0 : 'T', 21.0 : 'U', 22.0 : 'V', 23.0 : 'W', 24.0 : 'X', 27.0 : '-'}
class Profile(dbus.service.Object):
	fd = -1

	@dbus.service.method("org.bluez.Profile1",
					in_signature="", out_signature="")
	def Release(self):
		print("Release")
		mainloop.quit()

	@dbus.service.method("org.bluez.Profile1",
					in_signature="", out_signature="")
	def Cancel(self):
		print("Cancel")

	@dbus.service.method("org.bluez.Profile1",
				in_signature="oha{sv}", out_signature="")
	def NewConnection(self, path, fd, properties):
		self.fd = fd.take()
		print("NewConnection(%s, %d)" % (path, self.fd))
		#clf = joblib.load('RF_Trainer_10/trainer.pkl')
		clf = joblib.load('RF_Trainer_15/trainer.pkl')
		#clf = joblib.load('MLP_Trainer_1000/trainer.pkl')
		print("Trainer loaded")

		###################################################
		portNum = [8888, 7777, 6666, 5555, 4444]
		# create a socket object
		s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
		for i in portNum:
			server_address = ("127.0.0.1", i)
			# connection to hostname on the port.
			result = s.connect_ex(server_address)
			if not result:
				break		

		print("Result(%d)" % (result))
		if result!=0:
			print("Connection for tranmitting the data has not been initiated.")
			print("Bluetooth connection closed.")
			s.close()
			return False	
		###################################################


		server_sock = socket.fromfd(self.fd, socket.AF_UNIX,socket.SOCK_STREAM) 
		server_sock.setblocking(1)
		previousAlphabet = " "
		count = 0
		try:
			while True:
				try:
					#s.settimeout(2.0)
					start = time.time()
					tm = s.recv(69)

					if "close" in tm:
                                		s.close()
						server_sock.close()
						os.close(self.fd)
						self.fd = -1
						sys.exit()
					X = np.array(tm.split(), dtype=np.float)
					X = np.round(X,2)
					
					#Z = clf.predict([X])
					Z = float(clf.predict([X])[0])
					probability =clf.predict_proba([X])
					#print(max(probability[0]))
					
					if(max(probability[0])< 0.5):
						Z=27.0
					if Z == 27.0:
						count+=1
					print("Time taken :")
					print(time.time()-start)
					if previousAlphabet!=getAlphabet[Z] or Z==27.0:
						if Z==27.0 and count==30:
							server_sock.send(getAlphabet[Z])
							count=0
						elif Z!=27.00:
							server_sock.send(getAlphabet[Z])
						previousAlphabet = getAlphabet[Z]

					print(Z,getAlphabet[Z])
					
					#server_sock.settimeout(5.0)
					#server_sock.send(tm.decode('ascii'))
					#server_sock.settimeout(None)					
					#data = server_sock.recv(10)

				except socket.timeout:
					break
		except IOError:
			pass
		
		s.close()
		server_sock.close()
		print("all done")
		os.close(self.fd)
		self.fd = -1
		sys.exit()



	@dbus.service.method("org.bluez.Profile1",
				in_signature="o", out_signature="")
	def RequestDisconnection(self, path):
		print("RequestDisconnection(%s)" % (path))

		if (self.fd > 0):
			os.close(self.fd)
			self.fd = -1

if __name__ == '__main__':
	dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)

	bus = dbus.SystemBus()

	manager = dbus.Interface(bus.get_object("org.bluez",
				"/org/bluez"), "org.bluez.ProfileManager1")

	option_list = [
			make_option("-C", "--channel", action="store",
					type="int", dest="channel",
					default=None),
			]

	parser = OptionParser(option_list=option_list)

	(options, args) = parser.parse_args()

	options.uuid = "1101"
	options.psm = "3"
	options.role = "server"
	options.name = "eGlove"
	options.service = "spp char loopback"
	options.path = "/foo/bar/profile"
	options.auto_connect = False
	options.record = ""

	profile = Profile(bus, options.path)

	mainloop = GObject.MainLoop()

	opts = {
			"AutoConnect" :	options.auto_connect,
		}

	if (options.name):
		opts["Name"] = options.name

	if (options.role):
		opts["Role"] = options.role

	if (options.psm is not None):
		opts["PSM"] = dbus.UInt16(options.psm)

	if (options.channel is not None):
		opts["Channel"] = dbus.UInt16(options.channel)

	if (options.record):
		opts["ServiceRecord"] = options.record

	if (options.service):
		opts["Service"] = options.service

	if not options.uuid:
		options.uuid = str(uuid.uuid4())

	manager.RegisterProfile(options.path, options.uuid, opts)

	mainloop.run()