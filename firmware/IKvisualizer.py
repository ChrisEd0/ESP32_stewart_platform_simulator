import numpy as np
import matplotlib.pyplot as plt

# ----------------------------------------------------
#   DATOS FIJOS DE TU PLATAFORMA STEWART
# ----------------------------------------------------

servo_axis = np.array([
    [-62.804, -102.76, 15],
    [ 62.804, -102.76, 15],
    [120.395, -3.01,   15],
    [ 57.591, 105.77,  15],
    [-57.591, 105.77,  15],
    [-120.395,-3.01,   15]
])

servo_joint_init = np.array([
    [-79.774, -102.76, 31.971],
    [ 79.774, -102.76, 31.971],
    [128.88, -17.707, 31.971],
    [49.106, 120.467, 31.971],
    [-49.106, 120.467, 31.971],
    [-128.88, -17.707, 31.971]
])

platform_joint_init = np.array([
    [-10.25, -106.415, 186.047],
    [10.172, -106.182, 186.018],
    [99.206, 45.695, 185.875],
    [88.972, 62.962, 185.888],
    [-84.857, 60.971, 186.142],
    [-95.042, 42.855, 186.159]
])

ARM_LENGTH = 24
LIMIT = np.deg2rad(80)
z_axis = np.array([0, 0, 1])

# ----------------------------------------------------
#  FUNCIONES
# ----------------------------------------------------

def rotation_matrix(roll_deg, pitch_deg):
    roll = np.deg2rad(roll_deg)
    pitch = np.deg2rad(pitch_deg)

    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll),  np.cos(roll)]
    ])
    Ry = np.array([
        [ np.cos(pitch), 0, np.sin(pitch)],
        [0,              1,       0      ],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    return Ry @ Rx


def servo_plane_vectors(p0, p1):
    v = (p1 - p0)
    n = np.cross(v, z_axis)
    if np.linalg.norm(n) < 1e-6:
        n = np.cross(v, np.array([1,0,0]))
    n = n / np.linalg.norm(n)
    u = z_axis
    w = np.cross(n, u)
    w = w / np.linalg.norm(w)
    return u, w


BAR_LENGTH = 169.076  # longitud real de la barra

def solve_servo_angle(p0, current_p1, target_pos):
    """
    Resuelve el ángulo del servo de modo que:
    - La rótula caiga dentro del arco permitido
    - La barra mida exactamente BAR_LENGTH
    """

    # vectores del plano vertical del servo
    u, w = servo_plane_vectors(p0, current_p1)

    # ángulo inicial del servo (orientación del brazo actual)
    v = current_p1 - p0
    v = v / np.linalg.norm(v)
    x = np.dot(v, u)
    y = np.dot(v, w)
    theta0 = np.arctan2(y, x)

    # barrido de ángulos dentro del límite físico
    N = 2000
    thetas = np.linspace(theta0 - LIMIT, theta0 + LIMIT, N)

    best_theta = theta0
    best_err = 1e9
    best_p1 = current_p1

    for th in thetas:
        # posición de rótula de servo para ese ángulo
        candidate = p0 + ARM_LENGTH*(np.cos(th)*u + np.sin(th)*w)

        # distancia a rótula de plataforma
        dist = np.linalg.norm(candidate - target_pos)

        # error respecto a longitud real de barra
        err = abs(dist - BAR_LENGTH)

        # buscamos el error más bajo
        if err < best_err:
            best_err = err
            best_theta = th
            best_p1 = candidate

    return best_theta, best_p1



# ----------------------------------------------------
#  INGRESO ANGULOS
# ----------------------------------------------------

roll_in = float(input("Ingrese roll (°): "))
pitch_in = float(input("Ingrese pitch (°): "))

# ----------------------------------------------------
#  ROTAR LA PLATAFORMA
# ----------------------------------------------------

center = platform_joint_init.mean(axis=0)
R = rotation_matrix(roll_in, pitch_in)

platform_joints = (platform_joint_init - center) @ R.T + center

# ----------------------------------------------------
#  RESOLVER IK PARA CADA SERVO
# ----------------------------------------------------

servo_joint_new = np.zeros_like(servo_joint_init)
servo_angles = []

for i in range(6):
    theta, pos = solve_servo_angle(
        servo_axis[i],
        servo_joint_init[i],
        platform_joints[i]
    )
    servo_joint_new[i] = pos
    servo_angles.append(np.rad2deg(theta))


# ----------------------------------------------------
#  IMPRIMIR RESULTADOS
# ----------------------------------------------------

print("\n=== Ángulos de servos (°) ===")
for i, angle in enumerate(servo_angles):
    print(f"Servo {i+1} = {angle:.2f}°")

# ----------------------------------------------------
#  GRAFICAR RESULTADO (INICIAL Y FINAL)
# ----------------------------------------------------

fig = plt.figure(figsize=(10,10))
ax = fig.add_subplot(111, projection='3d')

# ------------------------
#  Ejes de servo (fijo)
# ------------------------
ax.scatter(servo_axis[:,0], servo_axis[:,1], servo_axis[:,2],
           c='blue', s=60, label='Eje de servo (fijo)')

# ------------------------
#  RÓTULAS DE SERVO (INICIAL)
# ------------------------
ax.scatter(servo_joint_init[:,0], servo_joint_init[:,1], servo_joint_init[:,2],
           c='red', s=50, label='Rótula servo inicial')

# ------------------------
#  RÓTULAS DE SERVO (FINAL)
# ------------------------
ax.scatter(servo_joint_new[:,0], servo_joint_new[:,1], servo_joint_new[:,2],
           c='orange', s=60, label='Rótula servo final')

# ------------------------
#  RÓTULAS DE PLATAFORMA (INICIAL)
# ------------------------
ax.scatter(platform_joint_init[:,0], platform_joint_init[:,1], platform_joint_init[:,2],
           c='magenta', s=70, label='Rótula plataforma inicial')

# ------------------------
#  RÓTULAS DE PLATAFORMA (FINAL)
# ------------------------
ax.scatter(platform_joints[:,0], platform_joints[:,1], platform_joints[:,2],
           c='green', s=70, label='Rótula plataforma final')

# ------------------------
#  Barras plataforma (INICIAL)
# ------------------------
for i in range(6):
    ax.plot(
        [servo_joint_init[i,0], platform_joint_init[i,0]],
        [servo_joint_init[i,1], platform_joint_init[i,1]],
        [servo_joint_init[i,2], platform_joint_init[i,2]],
        color='red', linestyle=':', linewidth=1
    )

# ------------------------
#  Barras plataforma (FINAL)
# ------------------------
for i in range(6):
    ax.plot(
        [servo_joint_new[i,0], platform_joints[i,0]],
        [servo_joint_new[i,1], platform_joints[i,1]],
        [servo_joint_new[i,2], platform_joints[i,2]],
        color='green', linewidth=2
    )

# ------------------------
#  Brazos de servo (INICIAL)
# ------------------------
for i in range(6):
    ax.plot(
        [servo_axis[i,0], servo_joint_init[i,0]],
        [servo_axis[i,1], servo_joint_init[i,1]],
        [servo_axis[i,2], servo_joint_init[i,2]],
        color='black', linestyle=':', linewidth=1
    )

# ------------------------
#  Brazos de servo (FINAL)
# ------------------------
for i in range(6):
    ax.plot(
        [servo_axis[i,0], servo_joint_new[i,0]],
        [servo_axis[i,1], servo_joint_new[i,1]],
        [servo_axis[i,2], servo_joint_new[i,2]],
        color='black', linewidth=1.5
    )

# ----------------------------------------------------
#  AJUSTES DE LA GRÁFICA
# ----------------------------------------------------

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_xlim(-200,200)
ax.set_ylim(-200,200)
ax.set_zlim(0,300)

ax.set_title("Plataforma Stewart – Poses Iniciales y Finales")
ax.legend()
ax.grid(True)

plt.show()

