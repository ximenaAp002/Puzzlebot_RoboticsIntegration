import rospy

def activate_node():
    # Inicializa el nodo si la condición se cumple
    rospy.init_node('mi_nodo')

def main():
    # Tu lógica aquí
    if condicion:
        activate_node()
    else:
        # No se inicializa el nodo si la condición no se cumple
        pass

if __name__ == '__main__':
    main()
