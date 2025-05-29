import matplotlib.pyplot as plt
import os

# Función para cargar datos desde un archivo de texto
def cargar_datos(nombre_archivo):
    with open(nombre_archivo, 'r') as f:
        contenido = f.read().strip()
    puntos = contenido.split(',')
    coordenadas = []
    for punto in puntos:
        try:
            x, y, _ = map(float, punto.strip().split())
            coordenadas.append((x, y))
        except:
            continue
    return coordenadas

# Cargar datos
datos1 = cargar_datos('odom.txt')

# Extraer coordenadas por separado
x1, y1 = zip(*datos1)

# Crear gráfico
plt.figure(figsize=(8, 8))
plt.plot(x1, y1, 'r-', label='Trayectoria turtlebot')

# Marcar los puntos objetivos
script_dir = os.path.dirname(os.path.realpath(__file__))
modo = "sqrt"
if modo == "sine":
    path = os.path.join(script_dir, "sine.txt")
elif modo == "sqrt":
    path = os.path.join(script_dir, "sqrt.txt")
else:
    path = os.path.join(script_dir, "line.txt")

with open(path) as f:
    data = f.readlines()

objetivos = []
for elem in data:
    elem = elem.strip("\n")
    elem = elem.split(",")
    objetivos.append([float(elem[0]), float(elem[1])])

# Extraer coordenadas por separado
x_obj, y_obj = zip(*objetivos)

# Dibujar todos los puntos objetivos en azul
plt.plot(x_obj, y_obj, 'b-', label=modo)

plt.legend()
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Trayectoria del turtlebot')
plt.axis('equal')
plt.grid(True)
plt.show()
