import sensor, image, time,pyb
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQQVGA)
sensor.skip_frames(time = 2000)
sensor.set_vflip(True)
sensor.set_hmirror(True)
uart = pyb.UART(3, 9600)
THRESHOLD = (0, 100, -128, -23, -128, 127)
clock = time.clock()
while(True):
	clock.tick()
	img = sensor.snapshot().binary([THRESHOLD])
	line = img.get_regression([(100,100)], robust = True)
	if (line):
		rho_err = abs(line.rho())-img.width()/2
		if line.theta()>90:
			theta_err = line.theta()-180
		else:
			theta_err = line.theta()
		img.draw_line(line.line(), color = 127)
		uart.write("E%.1f\r\n" % rho_err)
		print("偏差:%.1fpx" % rho_err)
	else:
		uart.write("S\r\n")
		print("未检测到黑线")