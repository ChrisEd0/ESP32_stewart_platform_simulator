import numpy as np
import time
import serial

puerto = "COM9"
baudrate = 115200

ser = serial.Serial(puerto, baudrate, timeout=1)
time.sleep(2)  # Esperar a que se inicie la conexión

import numpy as np
import time

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
BAR_LENGTH = 169.076


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
    """
    Devuelve los vectores unitarios u (vertical) y w (horizontal en el plano del servo).
    Se corre solo 1 vez por servo.
    """
    v = (p1 - p0)
    n = np.cross(v, z_axis)
    if np.linalg.norm(n) < 1e-6:
        n = np.cross(v, np.array([1,0,0]))
    n = n / np.linalg.norm(n)

    u = z_axis
    w = np.cross(n, u)
    w = w / np.linalg.norm(w)

    return u, w

def redondear(x):
    entero = int(x)
    decimal = abs(x - entero)
    if decimal >= 0.5:
        return entero + 1 +45 if x >= 0 else entero - 1 + 45
    else:
        return entero + 45
    
def empaquetar_y_enviar(angles, serial_port):
    """
    1. Redondea cada ángulo con redondear()
    2. Forma un string: ang,ang,ang,ang,ang,ang#
    3. Envía el string por el puerto serial
    """
    # 1. Redondear todos los ángulos
    angulos_redondeados = [redondear(a) for a in angles]

    # 2. Crear el string
    mensaje = ",".join(str(a) for a in angulos_redondeados) + "#"

    # 3. Enviar por el puerto serial
    serial_port.write(mensaje.encode())

    return mensaje  # útil para debug

# ----------------------------------------------------
#   SOLVER VECTORIAL OPTIMIZADO
# ----------------------------------------------------

def solve_servo_angle_fast(p0, current_p1, target_pos, u, w):
    """
    Versión vectorizada (sin bucles internos).
    """
    # Ángulo inicial
    v = current_p1 - p0
    v = v / np.linalg.norm(v)
    theta0 = np.arctan2(np.dot(v, w), np.dot(v, u))

    # Barrido vectorizado
    N = 400
    thetas = np.linspace(theta0 - LIMIT, theta0 + LIMIT, N)

    cos_t = np.cos(thetas)
    sin_t = np.sin(thetas)

    dirs = np.outer(cos_t, u) + np.outer(sin_t, w)   # (N,3)
    candidates = p0 + ARM_LENGTH * dirs              # (N,3)

    dists = np.linalg.norm(candidates - target_pos, axis=1)
    errors = np.abs(dists - BAR_LENGTH)

    idx = np.argmin(errors)
    return thetas[idx], candidates[idx]


# ----------------------------------------------------
#  PRE-CÁLCULO DE U/W PARA TODOS LOS SERVOS
# ----------------------------------------------------
servo_u = []
servo_w = []
for i in range(6):
    u, w = servo_plane_vectors(servo_axis[i], servo_joint_init[i])
    servo_u.append(u)
    servo_w.append(w)

servo_u = np.array(servo_u)
servo_w = np.array(servo_w)


# ----------------------------------------------------
#  INGRESO ANGULOS
# ----------------------------------------------------

# ----------------------------------------------------
#  BUCLE PRINCIPAL
# ----------------------------------------------------

while True:
    print("\n--------------------------------------")
    print("  INGRESE NUEVOS VALORES (o 'salir')")
    print("--------------------------------------")

    txt = input("Ingrese roll (°): ")

    if txt.lower() == "salir":
        break

    try:
        roll_in = float(txt)/0.89
        pitch_in = float(input("Ingrese pitch (°): "))/0.65
    except:
        print("⚠️ Entrada no válida. Intente de nuevo.")
        continue

    start = time.time()

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
        theta, pos = solve_servo_angle_fast(
            servo_axis[i],
            servo_joint_init[i],
            platform_joints[i],
            servo_u[i],
            servo_w[i]
        )
        servo_joint_new[i] = pos
        servo_angles.append(np.rad2deg(theta))

    end = time.time()

    # ----------------------------------------------------
    #  IMPRIMIR RESULTADOS
    # ----------------------------------------------------
    print("Tiempo:", end - start, "segundos")
    print("\n=== Ángulos de servos (°) ===")
    for i, angle in enumerate(servo_angles):
        ang_red = redondear(angle)
        print(f"Servo {i+1} = {angle:.2f}° → {ang_red}°  (normalizado)")

    # Enviar al ESP32
    mensaje_enviado = empaquetar_y_enviar(servo_angles, ser)
    print("\nMensaje serial enviado:", mensaje_enviado)

    # Pequeña pausa para estabilidad
    time.sleep(0.5)

# FUERA DEL WHILE
ser.close()
print("Puerto serial cerrado.")