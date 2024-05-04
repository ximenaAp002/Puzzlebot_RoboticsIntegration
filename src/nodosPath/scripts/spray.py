import os
import pygame
import time 
from pygame.locals import *

def mostrar_secuencia_gif(directorio):
    pygame.init()
    pantalla = pygame.display.set_mode((800, 480))
    background_color = (0, 0, 0)
    pantalla.fill(background_color)
    pygame.display.update()

    pygame.display.set_caption("Secuencia de imágenes")

    archivos = os.listdir(directorio)
    archivos_imagen = [archivo for archivo in archivos if archivo.endswith(('.png', '.jpg', '.jpeg'))]
    archivos_imagen.sort()  # Ordenar archivos de imagen

    index = 0
    corriendo = 0
    while corriendo < 16:
        for event in pygame.event.get():
            if event.type == QUIT:
                corriendo = False

        imagen_actual = pygame.image.load(os.path.join(directorio, archivos_imagen[index]))
        pantalla.blit(imagen_actual, (0, 0))
        pygame.display.flip()

        index = (index + 1) % len(archivos_imagen)
        pygame.time.delay(500)  # Retardo de 100 milisegundos (equivalente a 10 cuadros por segundo)
        corriendo +=1

    #pygame.quit()

opcion = 1

while True:

    if opcion == 1:
        directorio_caras_pixel_art = 'mushu\idle'
        mostrar_secuencia_gif(directorio_caras_pixel_art)
        opcion = 2
    elif opcion == 2:
        directorio_caras_pixel_art = 'mushu\TITE'
        mostrar_secuencia_gif(directorio_caras_pixel_art)
        opcion = 3
    elif opcion == 3:
        directorio_caras_pixel_art = 'mushu\enojao'
        mostrar_secuencia_gif(directorio_caras_pixel_art)
        opcion = 4 
    elif opcion == 4:
        directorio_caras_pixel_art = 'mushu\miaw'
        mostrar_secuencia_gif(directorio_caras_pixel_art)
        opcion = 5
    elif opcion == 5:
        directorio_caras_pixel_art = 'mushu\happy'
        mostrar_secuencia_gif(directorio_caras_pixel_art)
        opcion = 1
    else:
         print("Opción no válida. Por favor, elige una opción del menú.")

