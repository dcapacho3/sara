#!/usr/bin/env python3
# Autor: David Capacho Parra
# Fecha: Febrero 2025
# Descripción: Sistema de localización y registro de productos para SARA
# Implementa una interfaz que permite registrar productos en la base de datos
# utilizando la posición actual del robot, mostrando su ubicación en tiempo real
# en un mapa y permitiendo el control remoto para el posicionamiento.

import customtkinter as ctk
from PIL import Image
import os
import datetime
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import yaml
import threading
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import sqlite3
import math
import subprocess
from std_msgs.msg import String
import matplotlib.transforms as mtransforms
from ament_index_python.packages import get_package_share_directory
import time
import matplotlib.patches as patches

def get_source_db_path(package_name, db_filename):
    # Función para obtener la ruta de la base de datos en el directorio src del paquete
    # Navega desde el directorio share hasta la ubicación de la base de datos
    # siguiendo la estructura estándar de un workspace ROS2
    share_dir = get_package_share_directory(package_name)
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(share_dir))))
    db_path = os.path.join(workspace_root, 'src', package_name, 'database', db_filename)
    return db_path

def show_virtual_keyboard(entry_widget, parent, colors):
    # Función auxiliar para mostrar el teclado virtual
    # Crea una instancia del teclado y espera hasta que se cierre
    keyboard = CTkVirtualKeyboard(parent, entry_widget, colors)
    parent.wait_window(keyboard)

class CTkVirtualKeyboard(ctk.CTkToplevel):
    # Clase que implementa un teclado virtual para la entrada de texto
    # Facilita la interacción con la interfaz en dispositivos sin teclado físico
    def __init__(self, parent, entry_widget, colors):
        # Inicialización del teclado virtual
        # Configura la ventana emergente y sus controles
        super().__init__(parent)
        
        self.entry_widget = entry_widget
        self.colors = colors
        
        # Configuración de la ventana
        self.title("Teclado Virtual")
        self.configure(fg_color=colors['secondary_bg'])
        
        # Centrar la ventana
        window_width = 800
        window_height = 380
        screen_width = self.winfo_screenwidth()
        screen_height = self.winfo_screenheight()
        center_x = int(screen_width/2 - window_width/2)
        center_y = int(screen_height/2 - window_height/2)
        self.geometry(f'{window_width}x{window_height}+{center_x}+{center_y}')
        
        # Frame principal con padding
        self.main_frame = ctk.CTkFrame(self, fg_color=colors['secondary_bg'])
        self.main_frame.pack(expand=True, fill='both', padx=20, pady=20)
        
        # Mostrar el texto actual
        self.create_display()
        
        # Crear el teclado
        self.create_keyboard()
        
        # Hacer la ventana modal
        self.transient(parent)
        self.grab_set()
        
    def create_display(self):
        # Método para crear el área de visualización del texto
        # Muestra el texto que se está escribiendo actualmente
        # Frame para el display
        display_frame = ctk.CTkFrame(self.main_frame, fg_color=self.colors['primary_bg'])
        display_frame.pack(fill='x', pady=(0, 10))
        
        # Entry para mostrar el texto
        self.display = ctk.CTkEntry(
            display_frame,
            fg_color=self.colors['entry_bg'],
            text_color=self.colors['text_primary'],
            height=40,
            font=("Helvetica", 18)
        )
        self.display.pack(fill='x', padx=10, pady=10)
        self.display.insert(0, self.entry_widget.get())
        
    def create_keyboard(self):
        # Método para crear la disposición de teclas del teclado
        # Define el layout y crea los botones interactivos
        # Layout del teclado
        layouts = {
            'default': [
                ['1', '2', '3', '4', '5', '6', '7', '8', '9', '0'],
                ['Q', 'W', 'E', 'R', 'T', 'Y', 'U', 'I', 'O', 'P'],
                ['A', 'S', 'D', 'F', 'G', 'H', 'J', 'K', 'L', 'Ñ'],
                ['Z', 'X', 'C', 'V', 'B', 'N', 'M', '.', '-', '_']
            ],
            'symbols': [
                ['!', '@', '#', '$', '%', '^', '&', '*', '(', ')'],
                ['+', '=', '{', '}', '[', ']', '|', '\\', ':', ';'],
                ['<', '>', '?', '/', '"', "'", ',', '.', '~', '`'],
                ['¡', '¿', '°', '¬', '€', '£', '¥', '§', '¶', '©']
            ]
        }
        
        self.current_layout = 'default'
        
        # Frame para el teclado
        keyboard_frame = ctk.CTkFrame(self.main_frame, fg_color=self.colors['secondary_bg'])
        keyboard_frame.pack(expand=True, fill='both')
        
        # Crear filas para el teclado
        for row in layouts['default']:
            row_frame = ctk.CTkFrame(keyboard_frame, fg_color=self.colors['secondary_bg'])
            row_frame.pack(expand=True, fill='x', pady=2)
            
            for char in row:
                btn = ctk.CTkButton(
                    row_frame,
                    text=char,
                    command=lambda c=char: self.add_character(c),
                    width=60,
                    height=45,
                    fg_color=self.colors['button_bg'],
                    text_color=self.colors['button_text'],
                    hover_color=self.colors['button_hover'],
                    font=("Helvetica", 16)
                )
                btn.pack(side='left', padx=2, expand=True)
        
        # Frame para botones especiales
        special_frame = ctk.CTkFrame(keyboard_frame, fg_color=self.colors['secondary_bg'])
        special_frame.pack(expand=True, fill='x', pady=2)
        
        # Botones especiales
        special_buttons = [
            ("⌫ Borrar", self.backspace, 120),
            ("Espacio", lambda: self.add_character(' '), 300),
            ("123/#!", self.toggle_layout, 120),
            ("✓ Aceptar", self.accept, 120)
        ]
        
        for text, command, width in special_buttons:
            btn = ctk.CTkButton(
                special_frame,
                text=text,
                command=command,
                width=width,
                height=45,
                fg_color=self.colors['button_bg'],
                text_color=self.colors['button_text'],
                hover_color=self.colors['button_hover'],
                font=("Helvetica", 16)
            )
            btn.pack(side='left', padx=2, expand=True if text == "Espacio" else False)
            
    def add_character(self, char):
        # Método para añadir un carácter al campo de texto
        # Inserta el carácter en la posición actual del cursor
        current_pos = self.display.index(ctk.INSERT)
        self.display.insert(current_pos, char)
        
    def backspace(self):
        # Método para borrar el carácter anterior al cursor
        # Implementa la funcionalidad de retroceso
        current_pos = self.display.index(ctk.INSERT)
        if current_pos > 0:
            self.display.delete(current_pos - 1)
            
    def toggle_layout(self):
        # Método para alternar entre diferentes disposiciones del teclado
        # No implementado completamente, para futura expansión
        pass
        
    def accept(self):
        # Método para confirmar el texto ingresado
        # Transfiere el texto al campo de entrada original y cierra el teclado
        text = self.display.get()
        self.entry_widget.delete(0, 'end')
        self.entry_widget.insert(0, text)
        self.destroy()
        
    def cancel(self):
        # Método para cancelar la entrada de texto
        # Cierra el teclado sin transferir ningún texto
        self.destroy()

class SecondWindow(ctk.CTkToplevel):
    # Clase principal que implementa la ventana de localización de productos
    # Permite registrar productos utilizando la posición actual del robot
    def __init__(self, parent, mode, colors):
        # Inicialización de la ventana de localización
        # Configura la interfaz y establece la conexión con ROS
        super().__init__(parent)
        self.colors = colors
        self.mode = mode
        self.after_id = None
        self.current_pose = None
        self.is_closing = False
        self.ros_initialized = False
        self.launch_processes = []
        self.threads = []
        self.added_products = []
        self.calibration_complete = False
        
        # Configuración de la ventana
        self.setup_window()
        
        # Crear la interfaz
        self.setup_ui()
        
        # Iniciar el proceso de localización automáticamente
        self.start_thread(self.initialization_sequence)
        
        # Iniciar el reloj
        self.actualizar_reloj_y_fecha()
        self.colors = colors
        
        # Protocolo de cierre
        self.protocol("WM_DELETE_WINDOW", self.on_closing)

    def setup_window(self):
        # Método para configurar los parámetros de la ventana
        # Establece el título, tamaño y aspecto general
        self.title("Registro de Productos - SARA")
        self.geometry("%dx%d+0+0" % (self.winfo_screenwidth(), self.winfo_screenheight()))
        self.configure(fg_color=self.colors['primary_bg'])

    def setup_ui(self):
        # Método para configurar toda la interfaz de usuario
        # Crea y dispone los diferentes frames y widgets
        # Frame superior con reloj y fecha
        self.setup_top_frame()
        
        # Frame izquierdo con logo y controles
        self.setup_left_frame()
        
        # Frame central para el mapa
        self.setup_map_frame()
        
        # Frame derecho para la lista de productos
        self.setup_right_frame()

    def setup_top_frame(self):
        # Método para configurar el frame superior
        # Contiene botón de retorno, mensajes de estado y reloj/fecha
        self.top_frame = ctk.CTkFrame(self, height=100, fg_color=self.colors['secondary_bg'])
        self.top_frame.pack(fill=ctk.X, padx=10, pady=10)
        
        # Botón de retorno
        self.return_button = ctk.CTkButton(
            self.top_frame,
            text="Volver",
            command=self.return_to_main,
            font=('Arial', 16),
            fg_color=self.colors['button_bg'],
            text_color=self.colors['button_text'],
            hover_color=self.colors['button_hover'],
            width=100,
            height=40
        )
        self.return_button.pack(side=ctk.LEFT, padx=20)
        
        # Frame para el mensaje de estado
        self.status_frame = ctk.CTkFrame(self.top_frame, fg_color=self.colors['secondary_bg'])
        self.status_frame.pack(side=ctk.LEFT, padx=20, fill=ctk.X, expand=True)
        
        self.status_label = ctk.CTkLabel(
            self.status_frame,
            text="Iniciando sistema...",
            font=('Arial', 20, 'bold'),
            text_color=self.colors['text_primary']
        )
        self.status_label.pack(pady=10)
        
        # Frame para reloj y fecha
        clock_frame = ctk.CTkFrame(self.top_frame, fg_color=self.colors['secondary_bg'])
        clock_frame.pack(side=ctk.RIGHT, padx=20)
        
        self.label_fecha = ctk.CTkLabel(
            clock_frame,
            text="",
            font=('ARIAL', 25, 'bold'),
            text_color=self.colors['text_primary']
        )
        self.label_fecha.pack(side=ctk.TOP, pady=5)
        
        self.label_reloj = ctk.CTkLabel(
            clock_frame,
            text="",
            font=('ARIAL', 25, 'bold'),
            text_color=self.colors['text_primary']
        )
        self.label_reloj.pack(side=ctk.TOP, pady=5)

    def return_to_main(self):
        # Método para volver a la ventana principal
        # Muestra un diálogo de confirmación y cierra recursos antes de volver
        """Maneja el retorno a la ventana principal"""
        # Crear un diálogo personalizado
        dialog = ctk.CTkToplevel(self)
        dialog.title("Confirmar Retorno")
        dialog.geometry("400x150")
        dialog.configure(fg_color=self.colors['primary_bg'])
        
        # Centrar el diálogo
        x = self.winfo_x() + (self.winfo_width() - 400) // 2
        y = self.winfo_y() + (self.winfo_height() - 150) // 2
        dialog.geometry(f"+{x}+{y}")
        
        # Frame principal
        frame = ctk.CTkFrame(dialog, fg_color=self.colors['secondary_bg'])
        frame.pack(expand=True, fill="both", padx=20, pady=20)
        
        # Mensaje
        message = ctk.CTkLabel(
            frame,
            text="¿Está seguro que desea volver?\nSe cerrarán todos los procesos.",
            font=('Arial', 14),
            text_color=self.colors['text_primary']
        )
        message.pack(pady=20)
        
        # Función para manejar el OK
        def on_ok():
            dialog.destroy()
            # Limpiar todos los procesos
            self.cleanup()
            # Cerrar la ventana actual
            self.destroy()
            # Mostrar la ventana principal
            if self.master:
                self.master.deiconify()
        
        # Botón OK
        ok_button = ctk.CTkButton(
            frame,
            text="OK",
            command=on_ok,
            fg_color=self.colors['button_bg'],
            text_color=self.colors['button_text'],
            hover_color=self.colors['button_hover'],
            width=100
        )
        ok_button.pack(pady=10)
        
        # Asegurarse de que el diálogo esté completamente creado antes de hacerlo modal
        dialog.transient(self)
        
        # Esperar a que la ventana sea visible
        self.wait_visibility(dialog)
        
        # Ahora es seguro hacer el diálogo modal
        dialog.grab_set()
        
        # Esperar a que se cierre el diálogo
        self.wait_window(dialog)

    def setup_left_frame(self):
        # Método para configurar el frame izquierdo
        # Contiene logo, indicador de control y formulario de producto
        left_frame = ctk.CTkFrame(self, width=400, fg_color=self.colors['secondary_bg'])
        left_frame.pack(side=ctk.LEFT, fill=ctk.Y)
        left_frame.pack_propagate(False)
        
        # Logo
        self.setup_logo(left_frame)
        
        # Control status
        self.setup_control_status(left_frame)
        
        # Formulario para agregar productos
        self.setup_product_form(left_frame)
        
        # SARA label
        title_label = ctk.CTkLabel(
            left_frame,
            text="SARA",
            font=('Helvetica', 48, 'bold'),
            text_color=self.colors['text_primary']
        )
        title_label.pack(side=ctk.BOTTOM, pady=20)

    def setup_logo(self, parent_frame):
        # Método para configurar y mostrar el logo
        # Carga y muestra la imagen del logo en el frame proporcionado
        try:
            pkg_dir = get_package_share_directory('turtlemart')
            image_path = os.path.join(pkg_dir, 'images/userlogo.png')
            if os.path.exists(image_path):
                img = Image.open(image_path)
                size = 300
                img = img.resize((size, size), Image.LANCZOS)
                ctk_image = ctk.CTkImage(light_image=img, size=(size, size))
                logo_label = ctk.CTkLabel(parent_frame, image=ctk_image, text="")
                logo_label.pack(pady=30)
        except Exception as e:
            print(f"Error loading logo: {e}")

    def setup_control_status(self, parent):
        # Método para configurar el indicador de estado del control
        # Muestra visualmente si el control remoto está habilitado
        control_frame = ctk.CTkFrame(parent, fg_color="transparent")
        control_frame.pack(pady=20)
        
        self.control_label = ctk.CTkLabel(
            control_frame,
            text="Estado de Control Remoto",
            font=('Arial', 20, 'bold'),
            text_color=self.colors['text_primary']
        )
        self.control_label.pack(pady=(0, 10))
        
        self.control_canvas = ctk.CTkCanvas(
            control_frame,
            width=40,
            height=40,
            highlightthickness=0,
            bg=self.colors['secondary_bg']
        )
        self.control_canvas.pack()
        
        padding = 5
        self.control_circle = self.control_canvas.create_oval(
            padding, padding,
            40 - padding, 40 - padding,
            fill="red",
            outline=""
        )
        
        self.control_status_label = ctk.CTkLabel(
            control_frame,
            text="Deshabilitado",
            font=('Arial', 20),
            text_color=self.colors['text_primary']
        )
        self.control_status_label.pack(pady=5)

    def setup_product_form(self, parent):
        # Método para configurar el formulario de registro de productos
        # Crea los controles para ingresar nombre de producto y registrarlo
        form_frame = ctk.CTkFrame(parent, fg_color=self.colors['secondary_bg'])
        form_frame.pack(pady=20, padx=20, fill=ctk.X)
        
        name_label = ctk.CTkLabel(
            form_frame,
            text="Nombre del Producto:",
            font=('Arial', 16),
            text_color=self.colors['text_primary']
        )
        name_label.pack(pady=(10, 5))
        
        # Crear un frame para el campo de entrada y el botón del teclado
        entry_frame = ctk.CTkFrame(form_frame, fg_color="transparent")
        entry_frame.pack(pady=(0, 15))
        
        self.product_entry = ctk.CTkEntry(
            entry_frame,
            font=('Arial', 16),
            fg_color=self.colors['entry_bg'],
            width=200
        )
        self.product_entry.pack(side='left', padx=(0, 5))
        
        # Añadir botón de teclado
        keyboard_btn = ctk.CTkButton(
            entry_frame,
            text="⌨",
            command=lambda: show_virtual_keyboard(self.product_entry, self, self.colors),
            width=40,
            height=32,
            fg_color=self.colors['button_bg'],
            text_color=self.colors['button_text'],
            hover_color=self.colors['button_hover']
        )
        keyboard_btn.pack(side='left')
        
        self.register_button = ctk.CTkButton(
            form_frame,
            text="Registrar Producto",
            command=self.register_product,
            font=('Arial', 16),
            fg_color=self.colors['button_bg'],
            text_color=self.colors['button_text'],
            hover_color=self.colors['button_hover'],
            width=200,
            height=40,
            state="disabled"  # Comienza deshabilitado
        )
        self.register_button.pack(pady=10)

    def setup_map_frame(self):
        # Método para configurar el frame del mapa
        # Crea y configura el gráfico matplotlib para mostrar el mapa
        self.map_frame = ctk.CTkFrame(self, fg_color=self.colors['primary_bg'])
        self.map_frame.pack(side=ctk.LEFT, fill=ctk.BOTH, expand=True, padx=10, pady=10)
        
        self.fig, self.ax = plt.subplots(figsize=(8, 6))
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.map_frame)
        self.canvas.get_tk_widget().pack(fill=ctk.BOTH, expand=True)
        
        self.update_map()

    def setup_right_frame(self):
        # Método para configurar el frame derecho
        # Muestra la lista de productos registrados en la sesión actual
        self.right_frame = ctk.CTkFrame(self, width=350, fg_color=self.colors['secondary_bg'])
        self.right_frame.pack(side=ctk.RIGHT, fill=ctk.Y, padx=10, pady=10)
        
        title_label = ctk.CTkLabel(
            self.right_frame,
            text="Productos Agregados",
            font=('Helvetica', 24, 'bold'),
            text_color=self.colors['text_primary']
        )
        title_label.pack(pady=10)
        
        self.products_frame = ctk.CTkScrollableFrame(
            self.right_frame,
            fg_color=self.colors['scrollable_frame_bg'],
            width=300
        )
        self.products_frame.pack(fill=ctk.BOTH, expand=True, padx=10, pady=10)

    def initialization_sequence(self):
        # Método que implementa la secuencia de inicialización
        # Coordina el inicio de ROS, carga de mapas y preparación del sistema
        """Maneja la secuencia completa de inicialización"""
        try:
            # Mostrar mensaje inicial
            self.show_status_message("Iniciando sistema de localización...")
            
            # Primera fase: iniciar archivos ROS
            self.launch_ros2_files()
            self.show_status_message("Iniciando archivos del sistema...")
            
            # Esperar a que los archivos se inicien
            time.sleep(2)
            
            # Iniciar ROS en un thread separado
            self.ros_thread = threading.Thread(target=self.init_ros)
            self.ros_thread.daemon = True
            self.ros_thread.start()
            
            time.sleep(3)  # Dar tiempo a que ROS se inicialice
            # Marcar como completado y actualizar estado
            self.calibration_complete = True
            self.show_status_message("Sistema listo para registrar productos", is_final=True)
            
            # Habilitar registro de productos
            self.after(0, self.enable_product_registration)
            
        except Exception as e:
            self.show_error(f"Error en la inicialización: {str(e)}")

    def init_ros(self):
        # Método para inicializar ROS
        # Configura los nodos, suscripciones y ejecutor de ROS
        try:
            rclpy.init(args=None)
            self.ros_initialized = True
            self.node = rclpy.create_node('product_position_recorder')
            
            time.sleep(2.0)
            
            self.executor = rclpy.executors.SingleThreadedExecutor()
            self.executor.add_node(self.node)
            
            # Configurar suscripciones
            if self.mode == "Real":
                #self.odom_subscriber = self.node.create_subscription(
                #    PoseWithCovarianceStamped, 'amcl_pose', self.odom_callback, 10)
                self.odom_subscriber = self.node.create_subscription(
                    Odometry, 'odometry/filtered', self.odom_callback, 10)
            else:
                self.odom_subscriber = self.node.create_subscription(
                    Odometry, 'odom', self.odom_callback, 10)
            
            self.is_joy_on_subscriber = self.node.create_subscription(
                String, 'is_joy_on', self.is_joy_on_callback, 10)
            
            # Spin the executor in a loop
            while rclpy.ok() and not self.is_closing:
                try:
                    self.executor.spin_once(timeout_sec=0.1)
                except rclpy.executors.ExternalShutdownException:
                    break
                except Exception as e:
                    print(f"Error in ROS loop: {e}")
                    if self.is_closing:
                        break
                        
        except Exception as e:
            print(f"Error initializing ROS: {e}")
        finally:
            if self.is_closing:
                self.cleanup_ros()
                
    def cleanup_ros(self):
        # Método para limpiar recursos de ROS
        # Cierra nodos y detiene ROS cuando se cierra la aplicación
        if self.ros_initialized:
            if self.executor:
                self.executor.shutdown()
            if self.node:
                self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    def launch_ros2_files(self):
        # Método para lanzar los archivos necesarios de ROS2
        # Inicia mux, navegación y control básico en secuencia
        """Lanza los archivos ROS2 necesarios para la localización"""
        print("Iniciando secuencia de lanzamiento...")
        
        # 1. Primero lanzar el mux ya que es necesario para el control
        mux_thread = threading.Thread(target=self.launch_mux)
        mux_thread.start()
        time.sleep(2)  # Dar tiempo a que el mux se inicie
        
        # 2. Lanzar el nodo de navegación/localización
        if self.mode == "Real":
            nav_cmd = "ros2 launch turtlemart real_nav.launch.py map_yaml:=labrobfinal_mask.yaml"
        else:
            nav_cmd = "ros2 launch turtlemart navagv.launch.py map_yaml:=supermarket_map.yaml"
            
        nav_process = subprocess.Popen(
            nav_cmd,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT
        )
        self.launch_processes.append(nav_process)
        
        # Esperar a que el mapa se cargue
        time.sleep(2)
        
        # 3. Finalmente lanzar el control básico
        basic_control_thread = threading.Thread(target=self.launch_basic_control)
        basic_control_thread.start()

    def launch_mux(self):
        # Método para lanzar el multiplexor de control
        # Inicia el nodo de mux para el control del robot
        cmd = "ros2 launch turtlemart mux.launch.py"
        process = subprocess.Popen(
            cmd,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True
        )
        self.launch_processes.append(process)
        
        # Monitor the process output
        for line in process.stdout:
            print("Mux:", line.strip())
            
        process.wait()
        print("Mux launch file completed.")

    def launch_basic_control(self):
        # Método para lanzar el control básico
        # Inicia el nodo de control remoto del robot
        cmd = "ros2 launch turtlemart basic_control.launch.py"
        process = subprocess.Popen(
            cmd,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True
        )
        self.launch_processes.append(process)
        
        # Monitor the process output
        for line in process.stdout:
            print("Basic Control:", line.strip())
            
        process.wait()
        print("Basic Control launch file completed.")

    def setup_ros(self):
        # Método para configurar ROS
        # Inicializa ROS y configura suscripciones según el modo
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            self.ros_initialized = True
            self.node = rclpy.create_node('product_position_recorder')
            
            if self.mode == "Real":
                self.odom_subscriber = self.node.create_subscription(
                    PoseWithCovarianceStamped, 'amcl_pose', self.odom_callback, 10)
            else:
                self.odom_subscriber = self.node.create_subscription(
                    Odometry, 'odom', self.odom_callback, 10)
            
            self.is_joy_on_subscriber = self.node.create_subscription(
                String, 'is_joy_on', self.is_joy_on_callback, 10)
            
            self.executor = rclpy.executors.SingleThreadedExecutor()
            self.executor.add_node(self.node)
            
            self.ros_thread = threading.Thread(target=self.ros_spin)
            self.ros_thread.daemon = True
            self.ros_thread.start()
            
        except Exception as e:
            print(f"Error initializing ROS: {e}")

    def ros_spin(self):
        # Método para mantener activo el bucle de ROS
        # Procesa los mensajes de ROS mientras la aplicación está en ejecución
        while rclpy.ok() and not self.is_closing:
            try:
                self.executor.spin_once(timeout_sec=0.1)
            except Exception as e:
                print(f"Error in ROS spin: {e}")
                if self.is_closing:
                    break

    def show_status_message(self, message, is_final=False):
        # Método para mostrar mensajes de estado
        # Actualiza la etiqueta de estado con el mensaje proporcionado
        if not self.is_closing:
            self.after(0, lambda: self.status_label.configure(text=message))
            if is_final:
                self.after(2000, self.show_ready_message)

    def show_ready_message(self):
        # Método para mostrar un mensaje de sistema listo
        # Crea una ventana emergente informando que el sistema está listo para usar
        info_window = ctk.CTkToplevel(self)
        info_window.title("Sistema Listo")
        info_window.geometry("500x250")
        info_window.configure(fg_color=self.colors['primary_bg'])
        
        # Centrar la ventana
        window_width = 500
        window_height = 250
        screen_width = info_window.winfo_screenwidth()
        screen_height = info_window.winfo_screenheight()
        x = (screen_width - window_width) // 2
        y = (screen_height - window_height) // 2
        info_window.geometry(f"{window_width}x{window_height}+{x}+{y}")
        
        frame = ctk.CTkFrame(info_window, fg_color=self.colors['secondary_bg'])
        frame.pack(expand=True, fill="both", padx=20, pady=20)
        
        label = ctk.CTkLabel(
            frame,
            text="El sistema de localización está listo.\n\n" 
                 "Puede usar el control remoto para mover el robot por el mapa " \
                 "y registrar productos en las ubicaciones deseadas.\n\n" \
                 "La posición del robot se mostrará en tiempo real en el mapa.",
            font=('Arial', 14),
            text_color=self.colors['text_primary'],
            wraplength=400
        )
        label.pack(expand=True)
        
        button = ctk.CTkButton(
            frame,
            text="Entendido",
            command=info_window.destroy,
            fg_color=self.colors['button_bg'],
            text_color=self.colors['button_text'],
            hover_color=self.colors['button_hover']
        )
        button.pack(pady=10)

    def enable_product_registration(self):
        # Método para habilitar el registro de productos
        # Activa el botón de registro una vez que el sistema está listo
        """Habilita el registro de productos"""
        if hasattr(self, 'register_button'):
            self.register_button.configure(state="normal")

    def register_product(self):
        # Método para registrar un nuevo producto
        # Guarda el producto en la base de datos usando la posición actual del robot
        product_name = self.product_entry.get().strip()
        if not product_name:
            self.show_error("El nombre del producto no puede estar vacío")
            return
            
        if not self.current_pose:
            self.show_error("No se ha podido obtener la posición actual del robot")
            return
            
        try:
            db_dir = get_source_db_path('turtlemart', 'products.db')
            connection = sqlite3.connect(db_dir)
            cursor = connection.cursor()
            
            x = self.current_pose['x']
            y = self.current_pose['y']
            
            cursor.execute(
                "INSERT INTO products (name, x, y) VALUES (?, ?, ?)",
                (product_name, x, y)
            )
            connection.commit()
            
            # Agregar a la lista de productos de la sesión
            self.added_products.append({
                'name': product_name,
                'x': x,
                'y': y
            })
            
            # Actualizar la visualización
            self.update_products_list()
            self.update_map()
            
            # Limpiar el campo de entrada
            self.product_entry.delete(0, 'end')
            
            self.show_info(f"Producto '{product_name}' registrado exitosamente")
            
        except sqlite3.Error as e:
            self.show_error(f"Error al registrar el producto: {str(e)}")
        finally:
            if 'connection' in locals():
                connection.close()

    def update_products_list(self):
        # Método para actualizar la lista de productos
        # Muestra los productos agregados en la sesión actual
        # Limpiar la lista actual
        for widget in self.products_frame.winfo_children():
            widget.destroy()
            
        # Mostrar los productos agregados en esta sesión
        for product in self.added_products:
            product_frame = ctk.CTkFrame(
                self.products_frame,
                fg_color=self.colors['scrollable_frame_bg']
            )
            product_frame.pack(fill=ctk.X, padx=5, pady=2)
            
            label = ctk.CTkLabel(
                product_frame,
                text=f"{product['name']} ({product['x']:.2f}, {product['y']:.2f})",
                font=('Arial', 14),
                text_color=self.colors['text_primary']
            )
            label.pack(side=ctk.LEFT, padx=10, pady=5)

    def update_map(self):
        # Método para actualizar el mapa
        # Redibuja el mapa con la posición actual del robot y los productos
        self.ax.clear()
        map_array, resolution, origin = self.load_map()
        self.resolution = resolution
        self.origin = origin
        self.ax.imshow(np.flipud(map_array), cmap='gray', origin='lower')
        
        # Plotear productos agregados
        for product in self.added_products:
            pixel_x = int((product['x'] - origin[0]) / resolution)
            pixel_y = int((product['y'] - origin[1]) / resolution)
            self.ax.plot(pixel_x, pixel_y, 'ro', markersize=8)
            self.ax.annotate(product['name'], (pixel_x, pixel_y))
        
        # Plotear posición actual del robot con punto y arco direccional
        if self.current_pose:
            pixel_x = int((self.current_pose['x'] - origin[0]) / resolution)
            pixel_y = int((self.current_pose['y'] - origin[1]) / resolution)
            
            # Dibujar el punto central del robot
            self.ax.plot(pixel_x, pixel_y, 'bo', markersize=6)  # Punto ligeramente más pequeño
            
            # Configurar el arco direccional con radio más pequeño
            radius = 4  # Radio reducido del arco en píxeles
            # Convertir el ángulo de radianes a grados y ajustar para la orientación del arco
            angle_degrees = math.degrees(self.current_pose['orientation'])
            # El arco comienza 45 grados antes y termina 45 grados después del ángulo de orientación
            start_angle = angle_degrees - 45
            end_angle = angle_degrees + 45
            
            # Crear y añadir el arco
            arc = patches.Arc(
                (pixel_x, pixel_y),  # centro del arco
                radius * 2,  # ancho del arco
                radius * 2,  # altura del arco
                angle=0,  # rotación del arco completo
                theta1=start_angle,  # ángulo inicial
                theta2=end_angle,  # ángulo final
                color='blue',
                linewidth=2
            )
            self.ax.add_patch(arc)
        
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        self.canvas.draw()

    def load_map(self):
        # Método para cargar el mapa desde el archivo
        # Lee el archivo YAML del mapa y la imagen correspondiente
        bringup_dir = get_package_share_directory('turtlemart')
        if self.mode == "Real":
            map_yaml_path = os.path.join(bringup_dir, 'maps/labrobfinal_mask.yaml')
        else:
            map_yaml_path = os.path.join(bringup_dir, 'maps/supermarket_map.yaml')

        with open(map_yaml_path, 'r') as f:
            yaml_content = yaml.safe_load(f)

        resolution = yaml_content['resolution']
        origin = yaml_content['origin']
        map_image_path = yaml_content['image']

        if not os.path.isabs(map_image_path):
            map_image_path = os.path.join(os.path.dirname(map_yaml_path), map_image_path)

        map_image = Image.open(map_image_path)
        map_array = np.array(map_image)

        return map_array, resolution, origin

    def odom_callback(self, msg):
        # Callback para procesar mensajes de odometría
        # Actualiza la posición actual del robot y redibuja el mapa
        if self.mode == "Real":
            self.current_pose = {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'orientation': self.get_yaw_from_quaternion(msg.pose.pose.orientation)
            }
        else:
            self.current_pose = {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'orientation': self.get_yaw_from_quaternion(msg.pose.pose.orientation)
            }
        self.update_map()

    def is_joy_on_callback(self, msg):
        # Callback para procesar mensajes de estado del joystick
        # Actualiza el indicador visual según si el control está habilitado
        try:
            if msg.data == "yes":
                self.control_canvas.itemconfig(
                    self.control_circle,
                    fill="green",
                    outline=""
                )
                self.control_status_label.configure(text="Habilitado")
            else:
                self.control_canvas.itemconfig(
                    self.control_circle,
                    fill="red",
                    outline=""
                )
                self.control_status_label.configure(text="Deshabilitado")
            self.control_canvas.update_idletasks()
        except Exception as e:
            print(f"Error updating control circle color: {e}")

    def get_yaw_from_quaternion(self, quaternion):
        # Método para extraer el ángulo de giro (yaw) de un cuaternión
        # Convierte la representación de orientación en cuaternión a ángulo de guiñada
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return yaw

    def actualizar_reloj_y_fecha(self):
        # Método para actualizar el reloj y la fecha
        # Actualiza las etiquetas con la hora y fecha actuales cada segundo
        if not self.is_closing:
            now = datetime.datetime.now()
            self.label_reloj.configure(text=now.strftime("%H:%M:%S"))
            self.label_fecha.configure(text=now.strftime("%Y-%m-%d"))
            self.after_id = self.after(1000, self.actualizar_reloj_y_fecha)

    def show_error(self, message):
        # Método para mostrar mensajes de error
        # Crea y muestra una ventana emergente con el mensaje de error
        error_window = ctk.CTkToplevel(self)
        error_window.title("Error")
        error_window.geometry("400x200")
        error_window.configure(fg_color=self.colors['primary_bg'])
        
        frame = ctk.CTkFrame(error_window, fg_color=self.colors['secondary_bg'])
        frame.pack(expand=True, fill="both", padx=20, pady=20)
        
        label = ctk.CTkLabel(
            frame,
            text=message,
            font=('Arial', 14),
            text_color=self.colors['text_primary'],
            wraplength=300
        )
        label.pack(expand=True)
        
        button = ctk.CTkButton(
            frame,
            text="Aceptar",
            command=error_window.destroy,
            fg_color=self.colors['button_bg'],
            text_color=self.colors['button_text'],
            hover_color=self.colors['button_hover']
        )
        button.pack(pady=10)

    def show_info(self, message):
        # Método para mostrar mensajes informativos
        # Crea y muestra una ventana emergente con el mensaje informativo
        info_window = ctk.CTkToplevel(self)
        info_window.title("Información")
        info_window.geometry("400x200")
        info_window.configure(fg_color=self.colors['primary_bg'])
        
        frame = ctk.CTkFrame(info_window, fg_color=self.colors['secondary_bg'])
        frame.pack(expand=True, fill="both", padx=20, pady=20)
        
        label = ctk.CTkLabel(
            frame,
            text=message,
            font=('Arial', 14),
            text_color=self.colors['text_primary'],
            wraplength=300
        )
        label.pack(expand=True)
        
        button = ctk.CTkButton(
            frame,
            text="Aceptar",
            command=info_window.destroy,
            fg_color=self.colors['button_bg'],
            text_color=self.colors['button_text'],
            hover_color=self.colors['button_hover']
        )
        button.pack(pady=10)

    def start_thread(self, target, *args, **kwargs):
        # Método auxiliar para iniciar y rastrear hilos
        # Crea un hilo daemon y lo almacena para gestión posterior
        """Helper method to start and track threads"""
        thread = threading.Thread(target=target, args=args, kwargs=kwargs)
        thread.daemon = True
        self.threads.append(thread)
        thread.start()
        return thread

    def cleanup(self):
        # Método para limpiar todos los procesos y recursos
        # Realiza una limpieza exhaustiva antes de cerrar la aplicación
        """Método mejorado para limpiar todos los procesos"""
        try:
            # Marcar como cerrando para detener loops
            self.is_closing = True
            
            # Cancelar el actualizador del reloj
            if self.after_id is not None:
                self.after_cancel(self.after_id)
            
            # Limpiar procesos ROS
            if self.ros_initialized:
                if self.node:
                    self.node.destroy_node()
                rclpy.shutdown()
            
            # Terminar todos los procesos lanzados
            for process in self.launch_processes:
                try:
                    process.terminate()
                    process.wait(timeout=5)
                except:
                    try:
                        process.kill()
                    except:
                        pass
            
            # Limpiar la lista de procesos
            self.launch_processes.clear()
            
            # Esperar a que todos los hilos terminen
            for thread in self.threads:
                if thread.is_alive():
                    thread.join(timeout=1)
            
            # Limpiar la lista de hilos
            self.threads.clear()
            
            # Terminar procesos específicos de ROS
            processes_to_kill = [
                "rviz2",
                "gazebo",
                "gzclient",
                "gzserver",
                #"ros2",
               # "python3"  # Ten cuidado con este, solo matará los procesos python relacionados con ROS
            ]
            
            # Usar pkill para terminar los procesos
            for process_name in processes_to_kill:
                try:
                    # Primero intentar cerrar suavemente
                    subprocess.run(['pkill', '-TERM', process_name], check=False)
                    # Esperar un momento
                    time.sleep(0.5)
                    # Si aún está ejecutándose, forzar el cierre
                    subprocess.run(['pkill', '-9', process_name], check=False)
                except Exception as e:
                    print(f"Error killing {process_name}: {e}")
            
            # Asegurarse de que todos los nodos de ROS2 se detengan
            try:
                subprocess.run(['ros2', 'daemon', 'stop'], check=False)
            except Exception as e:
                print(f"Error stopping ROS2 daemon: {e}")
                
        except Exception as e:
            print(f"Error in cleanup: {e}")

    def on_closing(self):
        # Método para manejar el cierre de la ventana
        # Inicia el proceso de limpieza y cierre ordenado
        if hasattr(self, 'is_closing') and self.is_closing:
            return
            
        self.is_closing = True
        
        if self.after_id is not None:
            self.after_cancel(self.after_id)
            
        self.cleanup()
        self.destroy()

if __name__ == "__main__":
    # Punto de entrada principal cuando se ejecuta directamente
    # Crea la ventana de localización con el modo "Real"
    root = ctk.CTk()
    colors = {
        'primary_bg': "#FEF2F2",
        'secondary_bg': "#FEE2E2",
        'accent_bg': "#FFFFFF",
        'list_bg': "#F8FAFC",
        'frame_bg': "#FEE2E2",
        'scrollable_frame_bg': "#FFFFFF",
        'text_primary': "#7F1D1D",
        'text_secondary': "#DC2626",
        'label_text': "#B91C1C",
        'button_bg': "#DC2626",
        'button_hover': "#991B1B",
        'button_text': "#FFFFFF",
        'entry_bg': "#F8FAFC",
        'checkbox_bg': "#DC2626",
        'checkbox_hover': "#991B1B",
        'optionmenu_bg': "#F8FAFC",
        'optionmenu_button': "#DC2626",
        'optionmenu_hover': "#991B1B",
        'optionmenu_text': "#7F1D1D",
    }
    app = SecondWindow(root, "Real", colors)
    root.mainloop()