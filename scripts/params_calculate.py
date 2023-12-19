import math

PI = 3.14

pho = 1.225 # 空气密度
Kv = 900 # 电机Kv值
D = 0.28 # 桨叶直径 单位：米
Dc = 0.03 # 桨叶弧长
N = 2 # 桨叶数
theta = 1.62 * 3.14/180 # 螺旋桨角度 单位：弧度


# 先找到工作电流，查询电机和桨叶参数在该电流下的拉力及功率
U = 16.7 # 电池电压 单位：伏
I = 26.5  # 工作电流  单位：安培
T = 1650 * 0.0098 # 拉力 单位：牛
W_R = U*I - 371 # 电机热功率
angularVel_rp_m = 9000 # 螺旋桨角速度， 通过查看螺旋桨在提供T的拉力时的转速得到


Td = T*math.tan(theta)

angularVel_rad_s = angularVel_rp_m*2*PI/(60)
angularVel_rad_s_max = Kv*U
Kt = 60/(2*PI*Kv)
torque = Kt*I
Km = torque/((U*I - angularVel_rad_s * Td) ** 0.5)


Cd = 8*Td/(pho*(angularVel_rad_s ** 2)*Dc*N*((D/2)**4))
Cm = 8*Td*Dc/8/(pho*(angularVel_rad_s ** 2)*Dc*N*((D/2)**4))


print("maxRotVelocity: %s" % angularVel_rad_s_max)
print("motorConstant: %s" % Km)
print("momentConstant: %s" % Kt)
print("rotorDragCoefficient: %s" % Cd)
print("rollingMomentCoefficient: %s" % Cm)

