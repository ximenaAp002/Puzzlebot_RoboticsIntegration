#!/usr/bin/env python
import pygame
import time 
from pygame.locals import *
import rospy
import random
import os
from std_msgs.msg import Int16


def emoji_callback(emoji):
    global emojiGato
    emojiGato = emoji.data
    if emojiGato == 1:
        directorio_caras_pixel_art = '/home/karma/repoROS/src/nodosPath/mushu/idle'
        mostrar_secuencia_gif(directorio_caras_pixel_art)
    elif emojiGato == 2:
        directorio_caras_pixel_art = '/home/karma/repoROS/src/nodosPath/mushu/tite'
        mostrar_secuencia_gif(directorio_caras_pixel_art)
    elif emojiGato == 3:
        directorio_caras_pixel_art = '/home/karma/repoROS/src/nodosPath/mushu/enojao'
        mostrar_secuencia_gif(directorio_caras_pixel_art)
    elif emojiGato == 4:
        directorio_caras_pixel_art = '/home/karma/repoROS/src/nodosPath/mushu/miaw'
        mostrar_secuencia_gif(directorio_caras_pixel_art)
    elif emojiGato == 5:
        directorio_caras_pixel_art = '/home/karma/repoROS/src/nodosPath/mushu/happy'
        mostrar_secuencia_gif(directorio_caras_pixel_art)

    
def mostrar_secuencia_gif(directorio):
    pygame.init()
    pantalla = pygame.display.set_mode((800, 480))
    background_color = (0, 0, 0)
    pantalla.fill(background_color)
    pygame.display.update()

    pygame.display.set_caption("Secuencia de imagenes")

    archivos = os.listdir(directorio)
    archivos_imagen = [archivo for archivo in archivos if archivo.endswith(('.png', '.jpg', '.jpeg'))]
    archivos_imagen.sort()  # Ordenar archivos de imagen

    index = 0
    corriendo = 0
    ciclos = 4
    while corriendo < 4 * ciclos:
        for event in pygame.event.get():
            if event.type == QUIT:
                corriendo = False

        imagen_actual = pygame.image.load(os.path.join(directorio, archivos_imagen[index]))
        pantalla.blit(imagen_actual, (0, 0))
        pygame.display.flip()

        index = (index + 1) % len(archivos_imagen)
        pygame.time.delay(500)  # Retardo de 100 milisegundos (equivalente a 10 cuadros por segundo)
        corriendo +=1




if __name__=="__main__":
    try:
        rospy.init_node('mushu_face')
        nodeRate = 100
        rate = rospy.Rate(nodeRate)
        rospy.Subscriber("/mushuMode", Int16, emoji_callback)
        # if emojiGato == 0:
        #     emojiGato = random.randint(1,3)


        rospy.spin()
    except rospy.ROSInterruptException:
        pass


