#Recommendation: First, test with the ESP32 code to verify the servo's rotation conditions and the motor encoder connections to avoid damage to the structural and internal components of these devices.
import cv2
import numpy as np
import time
import math
import serial

# --- ROBOT CONFIGURATION ---
LONGITUD_L1 = 202.5 #length of the first link/articulation
LONGITUD_L2 = 255.0 #length of the second link/articulation
OFFSET_BASE_X = 11.6
OFFSET_BASE_Y = 37.0

# --- SERIAL CONFIGURATION ---
PUERTO_SERIAL = 'COM4'  # <--- CHECK YOUR PORT!, The same one where the ESP32 is connected
BAUD_RATE = 115200

# Straight Line Configuration
TIEMPO_RECORRIDO = 20  # Seconds to draw the line
PASOS_POR_SEGUNDO = 100 #dot plotting frequency

ser = None
try:
    ser = serial.Serial(PUERTO_SERIAL, BAUD_RATE, timeout=1)
    print(f"Conectado al ESP32 en {PUERTO_SERIAL}")
except Exception as e:
    print(f"Modo Simulación (Sin Serial): {e}")

# Global variables
detectando_punto = None
detectando_punto_coord = None 
detectando_x = None         
detectando_x_coord = None     

def nada(x): pass

# --- SEND FUNCTION (3 VALUES) ---
def enviar_datos(m1, m2, servo, etiqueta="Datos"):
    # FORMAT: "ANGLE1,ANGLE2,SERVO\n"
    mensaje = f"{m1:.2f},{m2:.2f},{int(servo)}\n" 
    
    if ser and ser.is_open:
        ser.write(mensaje.encode('utf-8'))
    else:
        pass

# --- 1. MATHEMATICS (IK) ---
def ordenar_puntos(puntos):
    rect = np.zeros((4, 2), dtype="float32")
    s = puntos.sum(axis=1)
    rect[0] = puntos[np.argmin(s)]
    rect[2] = puntos[np.argmax(s)]
    diff = np.diff(puntos, axis=1)
    rect[1] = puntos[np.argmin(diff)]
    rect[3] = puntos[np.argmax(diff)]
    return rect

def calcular_cinematica(x, y): #Inverse Kinematics Calculation
    if x == 0 and y == 0: return 0.0, 0.0
    r = math.sqrt(x**2 + y**2)
    if r > (LONGITUD_L1 + LONGITUD_L2): return None

    try:
        numerador = x**2 + y**2 - LONGITUD_L1**2 - LONGITUD_L2**2
        denominador = 2 * LONGITUD_L1 * LONGITUD_L2
        val = numerador / denominador
        cos_q2 = max(-1.0, min(1.0, val)) 
        q2_rad = math.acos(cos_q2)

        k1 = LONGITUD_L1 + LONGITUD_L2 * math.cos(q2_rad)
        k2 = LONGITUD_L2 * math.sin(q2_rad)
        q1_rad = math.atan2(y, x) - math.atan2(k2, k1)

        return math.degrees(q1_rad), math.degrees(q2_rad)
    except ValueError:
        return None

# --- 2. IMAGE PROCESSING ---
def procesar_contenido(frame, contorno_cuadrado, pts_ordenados, umbral_tinta):
    global detectando_punto, detectando_x
    global detectando_punto_coord, detectando_x_coord
    
    try:
        dst_pts = np.array([[300, 0], [0, 0], [0, 300], [300, 300]], dtype="float32")
        M_trans = cv2.getPerspectiveTransform(pts_ordenados, dst_pts)

        mask = np.zeros(frame.shape[:2], dtype="uint8")
        cv2.drawContours(mask, [contorno_cuadrado], -1, 255, -1)
        mask = cv2.erode(mask, np.ones((15, 15), np.uint8), iterations=1)

        gris = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        masked_gris = cv2.bitwise_and(gris, gris, mask=mask)
        _, thresh = cv2.threshold(masked_gris, umbral_tinta, 255, cv2.THRESH_BINARY_INV)
        thresh = cv2.bitwise_and(thresh, mask)

        contornos_internos, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contornos_internos:
            area = cv2.contourArea(cnt)
            if area > 50:
                hull = cv2.convexHull(cnt)
                if cv2.contourArea(hull) == 0: continue
                solidez = float(area) / cv2.contourArea(hull)

                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cx, cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
                    
                    pto_orig = np.array([[[cx, cy]]], dtype="float32")
                    pto_trans = cv2.perspectiveTransform(pto_orig, M_trans)
                    coord_x, coord_y = int(pto_trans[0][0][0]), int(pto_trans[0][0][1])

                    if coord_x < 15 or coord_x > 285 or coord_y < 15 or coord_y > 285: continue
                    if math.sqrt(coord_x**2 + coord_y**2) < 10: continue

                    tx_robot = max(0, min(300, coord_x)) + OFFSET_BASE_X
                    ty_robot = max(0, min(300, coord_y)) + OFFSET_BASE_Y
                    
                    angulos = calcular_cinematica(tx_robot, ty_robot)
                    texto_vis = f"{angulos[0]:.1f},{angulos[1]:.1f}" if angulos else "Out"

                    if solidez > 0.85: # PUNTO
                        if angulos: 
                            detectando_punto = angulos 
                            detectando_punto_coord = (tx_robot, ty_robot) 
                        (x,y), r = cv2.minEnclosingCircle(cnt)
                        cv2.circle(frame, (int(x), int(y)), int(r), (0, 255, 255), 1)
                        cv2.putText(frame, "P:" + texto_vis, (cx-30, cy-30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
                    
                    elif solidez < 0.82: # X
                        if angulos: 
                            detectando_x = angulos
                            detectando_x_coord = (tx_robot, ty_robot) 
                        rect = cv2.minAreaRect(cnt)
                        box = np.int32(cv2.boxPoints(rect))
                        cv2.drawContours(frame, [box], 0, (0, 0, 255), 1)
                        cv2.putText(frame, "X:" + texto_vis, (cx-30, cy-30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
    except Exception:
        pass

# --- 3. MAIN LOOP ---
def main():
    global detectando_punto, detectando_x, detectando_punto_coord, detectando_x_coord
    
    cap = cv2.VideoCapture(0) 
    time.sleep(2)
    if not cap.isOpened(): cap = cv2.VideoCapture(1)

    ventana = "Robot: Trazo con Levantado Final"
    cv2.namedWindow(ventana, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(ventana, 800, 600)
    
    cv2.createTrackbar("Brillo Fondo", ventana, 150, 255, nada)
    cv2.createTrackbar("Sensib. Tinta", ventana, 110, 255, nada)

    # State Variables
    fase = 0
    tiempo_ref = 0
    
    # Memory
    memoria_punto_ang = None
    memoria_punto_xy = None
    memoria_x_ang = None
    memoria_x_xy = None
    
    # Interpolation
    interpolacion_iniciada = False
    inicio_interpolacion_time = 0

    while True:
        ret, frame = cap.read()
        if not ret: break

        if fase == 0:
            detectando_punto = None
            detectando_x = None
            detectando_punto_coord = None
            detectando_x_coord = None

        gris = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        suave = cv2.GaussianBlur(gris, (3, 3), 0)
        val_fondo = cv2.getTrackbarPos("Brillo Fondo", ventana)
        _, mask = cv2.threshold(suave, val_fondo, 255, cv2.THRESH_BINARY)
        
        contornos, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contornos = sorted(contornos, key=cv2.contourArea, reverse=True)

        for c in contornos:
            if cv2.contourArea(c) > 2000:
                approx = cv2.approxPolyDP(c, 0.02 * cv2.arcLength(c, True), True)
                if len(approx) == 4:
                    pts_ord = ordenar_puntos(approx.reshape(4, 2))
                    cv2.drawContours(frame, [approx], -1, (0, 255, 0), 1)
                    val_tinta = cv2.getTrackbarPos("Sensib. Tinta", ventana)
                    procesar_contenido(frame, approx, pts_ord, val_tinta)
                    break 

        # --- CONTROL LOGIC ---
        tiempo_actual = time.time()
        mensaje = ""
        color_estado = (255, 255, 255)

        # PHASE 0: SEARCH
        if fase == 0:
            mensaje = "F0: BUSCANDO OBJETIVOS..."
            color_estado = (0, 0, 255) # Rojo
            if detectando_punto is not None and detectando_x is not None:
                mensaje = "OBJETIVOS OK. CONGELANDO..."
                memoria_punto_ang = detectando_punto
                memoria_punto_xy = detectando_punto_coord
                memoria_x_ang = detectando_x
                memoria_x_xy = detectando_x_coord
                tiempo_ref = tiempo_actual
                fase = 1
            elif detectando_punto: mensaje = "VEO PUNTO, FALTA X"
            elif detectando_x: mensaje = "VEO X, FALTA PUNTO"

        # PHASE 1: WAIT and INITIAL TRIP (SERVO 180)
        elif fase == 1:
            dt = 10 - int(tiempo_actual - tiempo_ref)
            mensaje = f"F1: PREPARANDO (SERVO 180)... {dt}s"
            color_estado = (0, 255, 255) # Amarillo
            if dt <= 0:
               # STARTING POINT -> Servo at 180, This depends on how the servo is mounted; it's recommended to test the rotation angles beforehand.
                enviar_datos(memoria_punto_ang[0], memoria_punto_ang[1], 180, "IR A PUNTO")
                tiempo_ref = tiempo_actual
                fase = 2
                interpolacion_iniciada = False

       # PHASE 2: TRACING SEQUENCE
        elif fase == 2:
            tiempo_desde_fase2 = tiempo_actual - tiempo_ref
            
            # --- SUB-STAGE A: ARRIVAL ---
            if tiempo_desde_fase2 < 4.0:
                 mensaje = "F2: A. LLEGANDO AL PUNTO... (SERVO 180)"
                 color_estado = (0, 255, 0)
            
            # --- SUB-STAGE B: PENCIL DROP ---
            elif tiempo_desde_fase2 < 10.0: 
                 dt_wait = int(10.0 - tiempo_desde_fase2)
                 mensaje = f"F2: B. BAJANDO LAPIZ... {dt_wait}s"
                 color_estado = (0, 0, 255) 
                 enviar_datos(memoria_punto_ang[0], memoria_punto_ang[1], 0, "BAJAR SERVO")
            
            # --- SUB-STAGE C: LINE DRAWING ---
            else:
                if not interpolacion_iniciada:
                    inicio_interpolacion_time = tiempo_actual
                    interpolacion_iniciada = True
                
                tiempo_transcurrido_linea = tiempo_actual - inicio_interpolacion_time
                progreso = tiempo_transcurrido_linea / TIEMPO_RECORRIDO
                if progreso > 1.0: progreso = 1.0

                start_x, start_y = memoria_punto_xy
                end_x, end_y = memoria_x_xy
                
                current_x = start_x + (end_x - start_x) * progreso
                current_y = start_y + (end_y - start_y) * progreso
                
                angulos_intermedios = calcular_cinematica(current_x, current_y)
                
                if angulos_intermedios:
                    # MOTION WITH SERVO 0 (Drawing)
                    enviar_datos(angulos_intermedios[0], angulos_intermedios[1], 0, "LINEA")
                
                mensaje = f"F2: C. TRAZANDO LINEA... {int(progreso*100)}%"
                color_estado = (0, 255, 0)

                # --- IMPORTANT MODIFICATION HERE ---
                if progreso >= 1.0:
                    # Before changing phase, we lift the pencil (180) 
                    # using the angles of the final position (the X)
                    ang_fin_1, ang_fin_2 = memoria_x_ang
                    enviar_datos(ang_fin_1, ang_fin_2, 180, "LAPIZ ARRIBA")
                    
                    tiempo_ref = tiempo_actual
                    fase = 3

        # PHASE 3: WAIT AND THEN RETURN TO ORIGIN
        elif fase == 3:
            dt = 10 - int(tiempo_actual - tiempo_ref)
            mensaje = f"F3: FIN TRAZO. ESPERANDO CON LAPIZ ARRIBA... {dt}s"
            color_estado = (0, 255, 0) 
            
            # Note: Since the servo has already been raised to the end of phase 2,
            # Here we just wait before moving to the origin.
            
            if dt <= 0:
                # HOME -> Servo at 180
                enviar_datos(0.0, 0.0, 180, "HOME")
                tiempo_ref = tiempo_actual
                fase = 4

        # PHASE 4: RESET
        elif fase == 4:
            dt = 20 - int(tiempo_actual - tiempo_ref)
            mensaje = f"F4: RESETEANDO... {dt}s"
            color_estado = (255, 0, 0) 
            if dt <= 0:
                fase = 0
                memoria_punto_ang = None
                memoria_x_ang = None

        cv2.rectangle(frame, (0, 0), (600, 50), (0, 0, 0), -1)
        cv2.putText(frame, mensaje, (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_estado, 2)
        
        cv2.imshow(ventana, frame)
        if cv2.waitKey(1) & 0xFF == ord('q'): break

    if ser and ser.is_open: ser.close()
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
