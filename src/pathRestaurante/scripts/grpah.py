import numpy as np
import matplotlib.pyplot as plt

# Suponiendo que tienes tus datos ya definidos


numPath = 1
ruta_archivo = "/home/karma/gazebo/src/pathRestaurante/paths/path" + str(numPath) + "Try1.txt"

datos = np.loadtxt(ruta_archivo, skiprows=1)
# Escalar las coordenadas
posiciones_x = .01 * datos[:, 0]
posiciones_y = .01 * datos[:, 1]

# Calcular las diferencias
diferenciaX = posiciones_x[0]
diferenciaY = posiciones_y[0]

for i in range(len(posiciones_x)):
    posiciones_x[i] -= diferenciaX
    posiciones_y[i] -= diferenciaY

# Revertir las coordenadas
posiciones_x_rev = np.flip(posiciones_x)
posiciones_y_rev = np.flip(posiciones_y)

print(posiciones_y)
print(posiciones_y_rev)

# Crear la figura y los ejes
plt.figure(figsize=(8, 6))
plt.plot(posiciones_x, posiciones_y, label='Original')
plt.plot(posiciones_x_rev, posiciones_y_rev, label='Revertido')
plt.xlabel('Posicin X')
plt.ylabel('Posicin Y')
plt.title('Grafico de Coordenadas')
plt.legend()
plt.grid(True)
plt.show()
