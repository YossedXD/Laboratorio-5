import pybullet as p
import pybullet_data
import time
import math

# Conexión con interfaz gráfica
physicsClient = p.connect(p.GUI)

# Rutas de datos
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Cargar el plano base
plane = p.loadURDF("plane.urdf")

# Cargar el modelo de cuadrúpedo (Laikago o Minitaur)
robot = p.loadURDF("laikago/laikago_toes.urdf", [0, 0, 0.5])

# Configurar gravedad
p.setGravity(0, 0, -9.8)

# Posicionar cámara
p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=50, cameraPitch=-30, cameraTargetPosition=[0, 0, 0.3])

# Número de articulaciones
num_joints = p.getNumJoints(robot)
print("Número de articulaciones:", num_joints)

# Control sencillo de marcha
t = 0
while True:
    for j in range(num_joints):
        angle = 0.5 * math.sin(t + j)
        p.setJointMotorControl2(robot, j, p.POSITION_CONTROL, targetPosition=angle)
    p.stepSimulation()
    t += 0.02
    time.sleep(1/240)
