#!/usr/bin/env python3
# Autor: David Capacho Parra
# Fecha: Febrero 2025
# Descripción: Creador de base de datos para el sistema de gestión de productos del robot SARA
# Implementa la configuración inicial de una base de datos SQLite con tablas para
# productos y selecciones de productos. Esta base de datos permite al robot
# SARA gestionar información de ubicaciones de productos en estanterías y
# listas de selección para tareas de recolección y navegación en supermercados.

import sqlite3
from ament_index_python.packages import get_package_share_directory
import os


def get_source_db_path(package_name, db_filename):
    """
    Obtiene la ruta a la base de datos en el directorio src del paquete
    """
    # Obtener el directorio share del paquete
    # Localiza la ubicación del paquete en el sistema de archivos
    share_dir = get_package_share_directory(package_name)
    
    # Navegar hasta la raíz del workspace (subir 4 niveles: share/package/install/workspace)
    # Determina la estructura del workspace para encontrar el directorio src
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(share_dir))))
    
    # Construir la ruta a la base de datos en src
    # Forma la ruta completa donde se almacenará la base de datos
    db_path = os.path.join(workspace_root, 'src', package_name, 'database', db_filename)
    
    #print(f"Trying to access database at: {db_path}")
    
    return db_path


def create_database():
    # Función principal para la creación de la base de datos
    # Establece la conexión y define el esquema de las tablas
    
    # Obtener las rutas necesarias para la base de datos
    # Utiliza funciones de ROS2 para localizar directorios del paquete
    bringup_dir = get_package_share_directory('sara')
    db_dir = get_source_db_path('sara', 'products.db')
    
    # Establecer conexión con la base de datos SQLite
    # Crea el archivo si no existe o se conecta si ya existe
    connection = sqlite3.connect(db_dir)
    cursor = connection.cursor()
    
    # Creación de tablas en la base de datos
    # Define la estructura de datos para la aplicación
    
    cursor.execute('''CREATE TABLE IF NOT EXISTS users (
                        user_id INTEGER PRIMARY KEY,
                        username TEXT NOT NULL,
                        password TEXT NOT NULL
                    )''')
    
    # Tabla de productos: almacena ubicaciones de productos en el mapa
    # Contiene las coordenadas para que el robot SARA pueda navegar hacia ellos
    cursor.execute('''CREATE TABLE IF NOT EXISTS products (
                        id INTEGER PRIMARY KEY,
                        name TEXT NOT NULL,
                        x REAL NOT NULL,
                        y REAL NOT NULL
                    )''')

    # Tabla de selecciones de productos: registra los productos seleccionados
    # Permite al robot SARA crear rutas de recolección de productos específicos

    cursor.execute('''CREATE TABLE IF NOT EXISTS user_selections (
                        selection_id INTEGER PRIMARY KEY,
                        user_id INTEGER,
                        product_id INTEGER,
                        FOREIGN KEY(user_id) REFERENCES users(user_id),
                        FOREIGN KEY(product_id) REFERENCES products(id)
                    )''')
    
    # Guardar cambios y cerrar la conexión
    # Asegura que la base de datos se almacene correctamente
    connection.commit()
    connection.close()

if __name__ == '__main__':
    create_database()
    print("Database and table created successfully.")