import numpy as np
from scipy.interpolate import interp1d
from scipy.integrate import simps  # Integración numérica
import os

def reparametrizar_curva(curva):
    curva = np.array(curva)
    dist = np.sqrt(np.sum(np.diff(curva, axis=0)**2, axis=1))
    s = np.insert(np.cumsum(dist), 0, 0)  # parámetro tipo "tiempo"
    return s, curva

def interpolar_curva(s, curva, s_uniforme):
    interp_func = interp1d(s, curva, axis=0, kind='linear', fill_value="extrapolate")
    return interp_func(s_uniforme)

def error_cuadratico_integrado(curva1, curva2, n=500):
    # Reparametrizar ambas curvas
    s1, pts1 = reparametrizar_curva(curva1)
    s2, pts2 = reparametrizar_curva(curva2)
    
    # Intervalo común
    s_min = max(s1[0], s2[0])
    s_max = min(s1[-1], s2[-1])
    s_uniforme = np.linspace(s_min, s_max, n)

    # Interpolar ambas curvas sobre el mismo dominio
    interp1 = interpolar_curva(s1, pts1, s_uniforme)
    interp2 = interpolar_curva(s2, pts2, s_uniforme)

    # Calcular diferencias al cuadrado
    errores = np.sum((interp1 - interp2)**2, axis=1)

    # Integrar con regla de Simpson
    integral_error = simps(errores, s_uniforme)
    return integral_error

# Ejemplo de uso:

#cargar datos
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

datos1 = cargar_datos('result_line.txt')

script_dir = os.path.dirname(os.path.realpath(__file__))
modo = "line"
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

def eliminar_puntos_repetidos(curva):
    curva_filtrada = [curva[0]]
    for p in curva[1:]:
        if not np.allclose(p, curva_filtrada[-1]):
            curva_filtrada.append(p)
    return curva_filtrada

def dar_vuelta_puntos(lista):
    result = []
    for elem in lista:
        result.append([elem[1], elem[0]])
    return result

def eliminar_bajos(lista, threshold):
    for i in range(len(lista)):
        elem = lista[i]
        post = lista[i+1]
        if elem[0] < post[0] and elem[1] > threshold:
            result = lista[i:]
            break
    return result

curva1 = datos1
curva2 = objetivos

curva1 = eliminar_puntos_repetidos(curva1)
curva2 = eliminar_puntos_repetidos(curva2)

curva1 = eliminar_bajos(curva1, 1.97)

print(curva1[0])

# print(curva1)
# print(curva2)

E = error_cuadratico_integrado(curva1, curva2)
print("Error cuadrático integrado:", E)
print("Error cuadrático medio:", E/(curva2[-1][0]-curva2[0][0]))
