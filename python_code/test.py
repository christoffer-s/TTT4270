import vel_pos

while True:
	vel, pos, yaw_ang, time, yaw_deg = vel_pos.vel_pos(vel, pos, yaw_ang, time)
	print("-"*30)
	print(pos)
	print("-"*30)
	print(vel)
	print("-"*30)
	print(yaw_deg)
	print("-"*30)