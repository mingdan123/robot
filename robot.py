# NACHI MZ07L 机器人正解计算 改进型DH模型正解换算
import numpy as np
import math
import copy
# 角度换算DH模型换算函数
def forward_kinematics(theta1, theta2, theta3, theta4, theta5, theta6):
    # DH参数
    d = [345.0, 0.0, 0.0, 440.0, 0.0, 73.0]
    a = [0.0, 50.0,420.0, 45.0, 0.0, 0.0]
    alpha = [0.0,90*math.pi/180, 0.0, 90*math.pi/180,-90*math.pi/180, 90*math.pi/180]
    # 将关节角度转换为弧度
    theta = np.radians([theta1, theta2, theta3, theta4, theta5, theta6])

    # 初始化变换矩阵为单位矩阵
    T = np.eye(4)

    # 计算正解
    for i in range(6):
        A = np.array([[np.cos(theta[i]), -np.sin(theta[i]), 0, a[i]],
                      [np.sin(theta[i])*np.cos(alpha[i]), np.cos(theta[i])*np.cos(alpha[i]), -np.sin(alpha[i]), -np.sin(alpha[i])*d[i]],
                      [np.sin(theta[i])*np.sin(alpha[i]), np.cos(theta[i])*np.sin(alpha[i]), np.cos(alpha[i]), np.cos(alpha[i])*d[i]],
                      [0, 0, 0, 1]])
        T = np.dot(T, A)


    # 获取末端执行器的位姿
    position = T[:3, 3]
    orientation = T[:3, :3]

    return position, orientation,T

# 输入关节角度
theta1 = 10
theta2 = 90
theta3 = 30
theta4 = 10
theta5 = -90
theta6 = 10
# 计算正解
position, orientation,TEXT = forward_kinematics(theta1, theta2, theta3, theta4, theta5, theta6)
# 求出RX RY RZ 需要在弧度下计算，完成后再矩阵转换成RX RY RZ角度
xysprt = math.sqrt(TEXT[0,0]*TEXT[0,0]+TEXT[1,0]*TEXT[1,0])
beta = math.atan2(-TEXT[2,0],xysprt)
aalpha = math.atan2(TEXT[1,0]/math.cos(beta),TEXT[0,0]/math.cos(beta))
gamma = math.atan2(TEXT[2,1]/math.cos(beta),TEXT[2,2]/math.cos(beta))
# 弧度转换成角度
RX = aalpha*180/math.pi
RY = beta*180/math.pi
RZ = gamma*180/math.pi
# 打印结果
print("末端执行器位置：", position)
print("正解矩阵", TEXT)
print("RX", RX)
print("RY", RY)
print("RZ", RZ)
print("=========================================================")
# 换算成法兰坐标换算成工具坐标
# 工具参数
#       X      Y      Z     RZ   RY    RX
tool = [10.0, 20.0, 30.0, 20.0, 30.0, 40.0]
# 转成弧度
tool_T = copy.copy(tool)
# tool_T=copy.deepcopy(tool)
# 对象的引用,此处会出现下表赋值的
# tool_T[5] = tool[3] * math.pi/180  tool[3]=tool_[3]造成计算错误
# tool_T=tool
# X和Z 参数需要交换
tool_T[3] = tool[5] * math.pi/180
tool_T[4] = tool[4] * math.pi/180
tool_T[5] = tool[3] * math.pi/180
print("工具坐标参数", tool_T)
# 转换矩阵
An = np.array([[np.cos(tool_T[3])*np.cos(tool_T[4]), np.cos(tool_T[3])*np.sin(tool_T[4])*np.sin(tool_T[5])-np.sin(tool_T[3])*np.cos(tool_T[5]), np.cos(tool_T[3])*np.sin(tool_T[4])*np.cos(tool_T[5])+np.sin(tool_T[3])*np.sin(tool_T[5]), tool_T[0]],
              [np.sin(tool_T[3]) * np.cos(tool_T[4]), np.sin(tool_T[3])*np.sin(tool_T[4])*np.sin(tool_T[5])+np.cos(tool_T[3])*np.cos(tool_T[5]), np.sin(tool_T[3])*np.sin(tool_T[4])*np.cos(tool_T[5])-np.cos(tool_T[3])*np.sin(tool_T[5]),tool_T[1]],
              [-np.sin(tool_T[4]), np.cos(tool_T[4]) * np.sin(tool_T[5]), np.cos(tool_T[4]) *np.cos(tool_T[5]),tool_T[2]],
              [0, 0, 0, 1]])
print("工具矩阵", An)
# 矩阵右乘
Tnnn = np.dot(TEXT, An)
print("工具坐标", Tnnn)
# 转成工具RPY 
Toolxysprt = math.sqrt(Tnnn[0,0]*Tnnn[0,0]+Tnnn[1,0]*Tnnn[1,0])
Toolbeta = math.atan2(-Tnnn[2,0],Toolxysprt)
Toolaalpha = math.atan2(Tnnn[1,0]/math.cos(Toolbeta),Tnnn[0,0]/math.cos(Toolbeta))
Toolgamma = math.atan2(Tnnn[2,1]/math.cos(Toolbeta),Tnnn[2,2]/math.cos(Toolbeta))
# TOOL TCP XYZ
Toolposition = Tnnn[:3, 3]
# TOOL TCP RPY
ToolRX = Toolaalpha*180/math.pi
ToolRY = Toolbeta*180/math.pi
ToolRZ = Toolgamma*180/math.pi
# 打印结果
print("TOOL位置：", Toolposition)
print("TOOLRX", ToolRX)
print("TOOLRY", ToolRY)
print("TOOLRZ", ToolRZ)