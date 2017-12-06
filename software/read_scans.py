import device

##

sdc=device.SeaDuckComm()


##

sdc.connect()
sdc.enter_command_mode(timeout=10)

print sdc.send('scan_period=400',timeout=2)
print sdc.send('scan'

scan_response=sdc.send('scan',timeout=10)

sdc.disconnect()
