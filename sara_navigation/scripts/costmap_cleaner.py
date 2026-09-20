#!/usr/bin/env python3
# Autor: David Capacho Parra
# Fecha: Febrero 2025
# Descripción: Limpiador de mapas de costo para el robot SARA
# Implementa un servicio periódico para limpiar el mapa de costo local
# utilizado por el sistema de navegación. Esto previene la acumulación
# de obstáculos fantasma y errores de detección, mejorando la fiabilidad
# de la navegación en entornos dinámicos.

import rclpy
from rclpy.node import Node
from nav2_msgs.srv import ClearEntireCostmap
import time

# Tiempo entre limpiezas en segundos
# Define la frecuencia con la que se eliminarán los datos del mapa de costo
CLEAN_INTERVAL = 1.2

class CostmapCleaner(Node):
    def __init__(self):
        # Inicialización del nodo ROS
        # Configura el cliente para llamar al servicio de limpieza del mapa de costo
        super().__init__('costmap_cleaner')
        self.client = self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
    
    def clear_costmap(self):
        # Método para solicitar la limpieza del mapa de costo
        # Crea y envía una solicitud vacía al servicio correspondiente
        request = ClearEntireCostmap.Request()
        self.client.call_async(request)

def main():
    # Función principal del programa
    # Inicializa el nodo y ejecuta la limpieza periódica hasta la interrupción
    rclpy.init()
    cleaner = CostmapCleaner()
    try:
        while True:
            # Bucle principal que realiza limpiezas periódicas
            # Llama al método de limpieza y espera el intervalo definido
            cleaner.clear_costmap()
            time.sleep(CLEAN_INTERVAL)
    except KeyboardInterrupt:
        # Manejo de la interrupción por teclado (Ctrl+C)
        # Permite una terminación limpia del programa
        pass
    
    # Limpieza final de recursos
    # Asegura que el nodo se destruya correctamente
    cleaner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()