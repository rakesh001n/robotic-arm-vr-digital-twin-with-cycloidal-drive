
import serial, time
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=2)
time.sleep(2)
ser.reset_input_buffer()
ser.write(b'<0,90,90,90,90>\n')
time.sleep(0.5)
print('sent')
resp = ser.read_all()
print('got:', resp)
ser.close()