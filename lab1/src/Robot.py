import random

# Tamaño de la matriz (5x5)
TAMANO = 5

# Símbolos para representar el mapa
LIBRE = "o"
OBSTACULO = "X"
INICIO = "I"
DESTINO = "D"
FLECHAS = {"arriba": "↑", "abajo": "↓", "izquierda": "←", "derecha": "→"}

# Generar una matriz de 5x5 con obstáculos aleatorios
def generar_matriz(tamano, densidad_obstaculos=0.2):
    matriz = [[LIBRE for _ in range(tamano)] for _ in range(tamano)]
    for i in range(tamano):
        for j in range(tamano):
            if random.random() < densidad_obstaculos:
                matriz[i][j] = OBSTACULO
    matriz[0][0] = INICIO  # Posición inicial del robot
    matriz[tamano - 1][tamano - 1] = DESTINO  # Posición del destino
    return matriz

# Muestra la matriz
def mostrar_matriz(matriz):
    for fila in matriz:
        print(" ".join(fila))
    print()

# Simula el movimiento del robot
def mover_robot(matriz):
    tamano = len(matriz)
    x, y = 0, 0  # Posición inicial
    ruta = []  # Se Almacena la ruta seguida
    direcciones = [("derecha", 0, 1), ("abajo", 1, 0), ("izquierda", 0, -1), ("arriba", -1, 0)]
    direccion_actual = 0

    while (x, y) != (tamano - 1, tamano - 1):
        # Intenta avanzar en la dirección actual
        dx, dy = direcciones[direccion_actual][1], direcciones[direccion_actual][2]
        nx, ny = x + dx, y + dy

        if 0 <= nx < tamano and 0 <= ny < tamano and matriz[nx][ny] != OBSTACULO:
            ruta.append((x, y, direcciones[direccion_actual][0]))  # Guarda la ruta
            x, y = nx, ny
        else:
            # Girar a la derecha (cambiar dirección)
            direccion_actual = (direccion_actual + 1) % 4

        # Si el robot vuelve al inicio, es imposible llegar al destino
        if (x, y) == (0, 0) and len(ruta) > 1:
            return None

    ruta.append((x, y, "destino"))  # Guardar la posición final
    return ruta

# Mostrar la ruta con flechas
def mostrar_ruta(matriz, ruta):
    mapa_ruta = [[LIBRE for _ in range(TAMANO)] for _ in range(TAMANO)]
    for paso in ruta:
        x, y, direccion = paso
        if direccion != "destino":
            mapa_ruta[x][y] = FLECHAS[direccion]
    mapa_ruta[TAMANO - 1][TAMANO - 1] = DESTINO
    mostrar_matriz(mapa_ruta)

# Programa principal
matriz = generar_matriz(TAMANO)
print("Mapa inicial con obstáculos:")
mostrar_matriz(matriz)

ruta = mover_robot(matriz)
if ruta:
    print("Ruta seguida por el robot:")
    mostrar_ruta(matriz, ruta)
else:
    print("Imposible llegar al destino")
