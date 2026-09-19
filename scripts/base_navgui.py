#!/usr/bin/env python3
# Autor: David Capacho Parra
# Fecha: Febrero 2025
# Descripción: Interfaz de navegación del sistema Smart Autonomous Retail Assistant (SARA)
# Implementa una ventana unificada que gestiona la navegación del robot tanto en modo
# real como en simulación, mostrando la posición del robot en un mapa y permitiendo
# al usuario interactuar con el sistema durante el proceso de compra.

import os
import sqlite3
import customtkinter as ctk
import datetime
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from PIL import Image
import numpy as np
import yaml
import threading
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped 
from nav_msgs.msg import Odometry
from robot_navigator import BasicNavigator, NavigationResult

from ament_index_python.packages import get_package_share_directory
from real_guiwaypoint import AutonomousNavigator as RealNavigator
from guiwaypoint import AutonomousNavigator as SimNavigator
from std_msgs.msg import String, Bool
import signal
import subprocess
import time
import math
import matplotlib.patches as patches
from matplotlib import transforms
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
from PIL import ImageOps 
import matplotlib.transforms as mtransforms
import sys
import tempfile
from finishgui import ThanksWindow



class UnifiedNavigationWindow(ctk.CTk):
    # Clase principal que implementa la interfaz de navegación unificada
    # Gestiona la interacción entre el usuario y el sistema de navegación del robot
    # proporcionando visualización en tiempo real y control del proceso de compra
    def __init__(self, product_manager,navigation_mode):
        # Inicialización de la ventana de navegación unificada
        # Configura la interfaz gráfica y establece la conexión con ROS2
        super().__init__()
        #self.master = master

        self.product_colors = {}
        self.product_labels = {}
        self.setup_signal_handlers()
        self.navigation_started = False  

        # Definición de fuentes para mantener consistencia en la interfaz
        self.STATUS_FONT = ('Arial', 25, 'bold')
        self.PRODUCT_FONT = ('Arial', 20, 'bold')
        self.PRODUCT_TITLE_FONT = ('Arial', 30, 'bold')      
        self.CLOCK_FONT = ('ARIAL', 25, 'bold')  # Para el reloj también

        # Paleta de colores para la interfaz
        # Define los colores usados en todos los elementos visuales
        self.colors = {
        # Fondos
        'primary_bg': "#FEF2F2",  # Rojo claro suave
        'secondary_bg': "#FEE2E2",  # Rosa coral vibrante
        'accent_bg': "#FFFFFF",  
        'list_bg': "#F8FAFC",  
        'frame_bg': "#FEE2E2",  
        'scrollable_frame_bg': "#FFFFFF",  
        # Texto
        'text_primary': "#7F1D1D",  # Rojo vino
        'text_secondary': "#DC2626",  # Rojo fuego
        'label_text': "#B91C1C",  # Rojo intenso
        # Elementos interactivos
        'button_bg': "#DC2626",  # Rojo llamativo
        'button_hover': "#991B1B",  # Rojo oscuro
        'button_text': "#FFFFFF",  
        'entry_bg': "#F8FAFC",  
        'checkbox_bg': "#DC2626",  
        'checkbox_hover': "#991B1B",  
        # Menú
        'optionmenu_bg': "#F8FAFC",  
        'optionmenu_button': "#DC2626",  
        'optionmenu_hover': "#991B1B",  
        'optionmenu_text': "#7F1D1D", 
        }

        self.configure(fg_color=self.colors['primary_bg'])
        self.after_id = None
        
        self.is_closing = False
        self.ros_initialized = False
        self.threads = []
        self.lock = threading.Lock()
        
        # Configuración de la interfaz
        self.product_manager = product_manager
        self.nav_mode = navigation_mode 
         
        # Configuración de la ventana principal
        self.title("Smart Autonomous Retail Assistant")
        self.geometry("%dx%d+0+0" % (self.winfo_screenwidth(), self.winfo_screenheight()))

        #self.attributes('-fullscreen', True) 
        #self.overrideredirect(True)  

        self.resizable(width=1, height=1)

        # Inicializar la lista de productos seleccionados
        self.selected_products = []

        # Variables para el seguimiento de la posición del robot
        self.initial_pose = None  # To store the starting x, y, and yaw
        self.current_pose = None  # To store the relative x, y, and yaw (yaw remains absolute)
        self.pose_message_printed = False  # Solo agregar esta variable

        # Variables para la comunicación con ROS
        self.node = None
        self.executor = None
        self.navigator = None
        self.odom_subscriber = None
        self.is_joy_on_subscriber = None
        self.continue_nav_publisher = None
        self.current_pose = None
        self.cashier_reached = False
        self.lock_all_active = False  # Agregar esta variable de estado
        self.should_show_popup = True
        self.current_popup = None  # Mantener referencia al popup actual

        # Deshabilitar cierre de ventana mediante botón X
        self.protocol("WM_DELETE_WINDOW", lambda: None)

        # Variables de estado para la calibración y navegación
        self.calibration_complete = False 
        
        self.launch_processes = []
        self.launch_thread = None
        
        # Iniciar hilo para la comunicación con ROS
        self.ros_thread = threading.Thread(target=self.init_ros)
        self.ros_thread.daemon = True  # Daemon para que termine cuando la GUI se cierre
        self.ros_thread.start()
        
        # Variables para la visualización del robot en el mapa
        self.robot_patch = None
        self.robot_width = 0.15  # in meters
        self.robot_length = 0.35  # in meters
        self.robot_image = None
        self.robot_imobj = None
        self.robot_artist = None
        self.cashier_marker = None  # New attribute to store the cashier marker
        self.fixed_cash_location = {'x': 0.1, 'y': 2.0} if self.nav_mode == "Real" \
            else {'x': -1.0, 'y': -2.0}

        # Variables para controlar el estado de la navegación
        self.navigation_started = False
        self.go_to_cashier= False
        self.continue_nav_published = False  # Nueva variable para controlar la publicación

        # Frame superior con información de estado
        self.top_frame = ctk.CTkFrame(self, height=100, fg_color=self.colors['secondary_bg'])
        self.top_frame.pack(side=ctk.TOP, fill=ctk.X, padx=10, pady=10)
        self.top_frame.pack_propagate(False)

        clock_frame = ctk.CTkFrame(self.top_frame, fg_color=self.colors['secondary_bg'])
        clock_frame.pack(side=ctk.RIGHT, padx=20)

        status_center_frame = ctk.CTkFrame(self.top_frame, fg_color=self.colors['secondary_bg'])
        status_center_frame.pack(side=ctk.LEFT, expand=True, fill=ctk.BOTH, padx=20)
        status_center_frame.pack_propagate(False) 
        
        # Etiqueta para mostrar el estado actual de la navegación
        self.status_label = ctk.CTkLabel(
            status_center_frame, 
            text="Por favor empiece con el proceso de compra. Haga clic en el boton 'Iniciar'",
            font=self.STATUS_FONT,
            text_color=self.colors['text_primary'],
            wraplength=1400
        )
        self.status_label.pack(pady=(10, 5))

        # Barra de progreso para mostrar el avance de la navegación
        self.progress_bar = ctk.CTkProgressBar(status_center_frame, width=700, progress_color=self.colors['button_bg'], fg_color=self.colors['secondary_bg'])
        self.progress_bar.pack(pady=(10, 20))
        self.progress_bar.set(0)

        # Frame para la información lateral (fecha, hora, etc.)
        self.info_frame = ctk.CTkFrame(self, width=200, fg_color=self.colors['secondary_bg'])
        self.info_frame.pack(side=ctk.LEFT, fill=ctk.Y, padx=10, pady=10)

        # Etiquetas de reloj y fecha
        self.label_fecha = ctk.CTkLabel(
            clock_frame, 
            text="",  # Se actualizará con actualizar_reloj_y_fecha
            font=self.CLOCK_FONT,
            text_color=self.colors['text_primary'],
        )
        self.label_fecha.pack(side=ctk.TOP, pady=5)

        self.label_reloj = ctk.CTkLabel(
            clock_frame, 
            text="",
            font=self.CLOCK_FONT,
            text_color=self.colors['text_primary']
        )
        self.label_reloj.pack(side=ctk.TOP, pady=5)

        # Etiqueta SARA en la parte inferior izquierda
        shop_vision_label = ctk.CTkLabel(self.info_frame, text="SARA", font=('Helvetica', 35, 'bold'), text_color=self.colors['text_primary'],)
        shop_vision_label.pack(side=ctk.BOTTOM, padx=10, pady=10)

        # Iniciar la actualización del reloj y la fecha
        self.actualizar_reloj_y_fecha()

        # Frame para el mapa de navegación
        self.map_frame = ctk.CTkFrame(self, width=800, height=600, fg_color=self.colors['primary_bg'])
        self.map_frame.pack(side=ctk.LEFT, fill=ctk.BOTH, expand=True, padx=10, pady=10)

        # Frame para los controles laterales derechos
        self.right_frame = ctk.CTkFrame(self, width=450,  fg_color=self.colors['secondary_bg'])
        self.right_frame.pack(side=ctk.RIGHT, fill=ctk.Y, padx=10, pady=10, expand=False)
        self.right_frame.pack_propagate(False) 

        # Frame para la lista de productos seleccionados
        self.selected_frame = ctk.CTkFrame(self.right_frame, width=350, fg_color=self.colors['list_bg'])
        self.selected_frame.pack(side=ctk.TOP, fill=ctk.BOTH, expand=True, padx=10, pady=10)
        
        # Frame para el botón de navegación
        self.button_frame = ctk.CTkFrame(self.right_frame, width=120, fg_color=self.colors['secondary_bg'])
        self.button_frame.pack(side=ctk.BOTTOM, fill=ctk.X, padx=10, pady=10)

        # Crear un marco interno para el botón con más altura
        self.button_inner_frame = ctk.CTkFrame(self.button_frame, height=100, width=100, fg_color=self.colors['secondary_bg'])
        self.button_inner_frame.pack(side=ctk.LEFT, fill=ctk.BOTH, expand=True, padx=10, pady=10)
   
        # Botón principal para controlar la navegación
        self.navigation_button = ctk.CTkButton(
            self.button_inner_frame,
            text="Iniciar",
            command=self.handle_navigation_button,        
            height=90,  # Mantenemos la altura del botón original
            fg_color=self.colors['button_bg'],
            text_color=self.colors['button_text'],
            hover_color=self.colors['button_hover'],
            font=self.PRODUCT_FONT
        )
        self.navigation_button.pack(side=ctk.LEFT, fill=ctk.BOTH, expand=True, padx=5)

        # Frame para mostrar el estado del control remoto
        self.control_frame = ctk.CTkFrame(self.info_frame, width=200, height=150, fg_color="transparent")
        self.control_frame.pack(side=ctk.TOP, padx=10, pady=20, expand=True)
        self.control_frame.pack_propagate(False)
        container = ctk.CTkFrame(self.control_frame, fg_color="transparent")
        container.place(relx=0.5, rely=0.5, anchor="center")

        # Etiqueta para el estado del control remoto
        self.control_label = ctk.CTkLabel(
            container, 
            text="Estado de Control Remoto",
            font=self.PRODUCT_FONT,
            text_color=self.colors['text_primary'],
            fg_color="transparent",
            wraplength=200
        )
        self.control_label.pack(pady=(0, 10))

        # Canvas para el indicador visual del control remoto
        self.control_canvas = ctk.CTkCanvas(
            container, 
            width=40, 
            height=40, 
            highlightthickness=0,
            bg=self.colors['secondary_bg']
        )
        self.control_canvas.pack(pady=(5, 0))

        # Crear el círculo indicador con padding
        padding = 5
        self.control_circle = self.control_canvas.create_oval(
            padding, 
            padding, 
            40 - padding, 
            40 - padding, 
            fill="red", 
            outline=""
        )
        
        # Etiqueta de texto para el estado del control
        self.control_status_label = ctk.CTkLabel(
            container,
            text="Deshabilitado",  # Texto inicial
            font=self.PRODUCT_FONT,
            text_color=self.colors['text_primary'],
            fg_color="transparent"
        )
        self.control_status_label.pack(pady=(5, 0))

        # Crear y mostrar el gráfico del mapa
        self.create_plot(self.map_frame)
        
        # Iniciar la actualización de la posición del robot
        self.update_robot_position()

        # Actualizar la lista de productos seleccionados
        self.view_selected_products()
        self.after(1000, self.view_selected_products)

    def init_ros(self):
        # Método para inicializar la comunicación con ROS
        # Configura los nodos, suscriptores y publicadores necesarios
        try:
            rclpy.init(args=None)
            self.ros_initialized = True
            self.node = rclpy.create_node('navigate_node')

            time.sleep(2.0)

            self.executor = rclpy.executors.SingleThreadedExecutor()
            self.executor.add_node(self.node)
            self.navigator = BasicNavigator()

            # Configuración de suscriptores según el modo de navegación
            if self.nav_mode == "Real":
               # self.odom_subscriber = self.node.create_subscription(
                #    PoseWithCovarianceStamped, 'amcl_pose', self.odom_callback, 10)
                self.odom_subscriber = self.node.create_subscription(
                    Odometry, 'odometry/filtered', self.odom_callback, 10)

                self.lock_all_subscriber = self.node.create_subscription(
                    Bool, 'lock_all', self.lock_all_callback, 10)
            else:
                self.odom_subscriber = self.node.create_subscription(
                    Odometry, 'odom', self.odom_callback, 10)

            # Suscriptor para el estado del joystick
            self.is_joy_on_subscriber = self.node.create_subscription(String, 'is_joy_on', self.is_joy_on_callback, 10) 

            # Publicadores para control de navegación
            self.continue_nav_publisher = self.node.create_publisher(String, '/continue_nav', 10)
            self.cashier_publisher = self.node.create_publisher(String, '/to_do_next', 10)
            self.status_subscriber = self.node.create_subscription(String, '/navigation_status', self.status_callback, 10)
        
            # Bucle principal de ejecución de ROS
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

    def odom_callback(self, msg):
        # Callback para actualizar la posición del robot
        # Procesa los mensajes de odometría para obtener la posición y orientación
        self.current_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'orientation': self.get_yaw_from_quaternion(msg.pose.pose.orientation)
        }

    def is_joy_on_callback(self, msg):
        # Callback para actualizar el estado del control remoto
        # Actualiza el indicador visual según si el joystick está activo
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
            # Force a redraw of the canvas to ensure immediate update
            self.control_canvas.update_idletasks()
        except Exception as e:
            print(f"Error updating control circle color: {e}")

    def lock_all_callback(self, msg):
        # Callback para manejar el bloqueo de la plataforma
        # Muestra un mensaje cuando se excede el peso máximo permitido
        if msg.data and self.should_show_popup:  # Si es True y debemos mostrar el popup
            self.show_info("Peso máximo de plataforma excedida, por favor retire el último producto agregado", "¡Atención!")
            self.should_show_popup = False
        elif not msg.data:  # Si es False
            if self.current_popup is not None:
                self.current_popup.destroy()  # Cerrar el popup si existe
                self.current_popup = None
            self.should_show_popup = True  # Permitir que se muestre el próximo True
        
        self.lock_all_active = msg.data

    def status_callback(self, msg):
        # Callback para procesar los mensajes de estado de la navegación
        # Actualiza la interfaz según el estado actual de la navegación

        status, waypoint_name, completion_percentage, visited_waypoints = msg.data.split('|')
        visited_waypoints = set(visited_waypoints.split(','))
        self.handle_progress_bar(status, waypoint_name, completion_percentage)
        self.update_waypoint_status(status, waypoint_name, visited_waypoints)
        
        # Llamar a handle_status para mostrar mensajes amigables
        self.handle_status(status, waypoint_name)
        
        # Actualizar el texto del botón según el estado
        if status == "READY":
            self.navigation_button.configure(
                text="Siguiente producto",
                state="normal"
            )
        elif status == "WAITING":
            self.navigation_button.configure(
                state="normal"
            )
        elif status == "FINISHED":
            self.navigation_button.configure(
                text="Ir a caja",
                state="normal"
            )
            self.go_to_cashier = True
            
        if status == "REACHED" and waypoint_name == "cashier":
            self.cashier_reached = True
        
            
    def update_waypoint_status(self, status, waypoint_name, visited_waypoints):
        # Método para actualizar el estado visual de los puntos de navegación
        # Cambia los colores de los marcadores en el mapa y la lista de productos
        locations = self.get_product_locations()
        
        for loc in locations:
            if loc['name'] == waypoint_name:
                pixel_x = int((loc['x'] - self.origin[0]) / self.resolution)
                pixel_y = int((loc['y'] - self.origin[1]) / self.resolution)
                
                if status == "REACHED":
                    self.update_marker_color(pixel_x, pixel_y, 'green')
                    self.update_product_color(waypoint_name, "#00FF00")  # Verde brillante
                elif status == "NAVIGATING":
                    self.update_marker_color(pixel_x, pixel_y, 'yellow')
                    self.update_product_color(waypoint_name, "#FFD700")  # Amarillo

    def update_product_color(self, product_name, color):
        # Método para actualizar el color de un producto en la lista
        # Cambia el color del texto sin recrear la lista completa
        self.product_colors[product_name] = color
        if product_name in self.product_labels:
            label = self.product_labels[product_name]
            label.configure(text_color=color)

    def update_marker_color(self, x, y, color):
        # Método para actualizar el color de un marcador en el mapa
        # Busca el marcador correspondiente y cambia su color
        for marker in self.ax.lines:
            if marker.get_xdata() == x and marker.get_ydata() == y:
                marker.set_color(color)
        self.canvas.draw()

    def update_text_color(self, name, color):
        # Método para actualizar el color del texto de un producto
        # Busca la etiqueta correspondiente dentro de los contenedores
        self.product_colors[name] = color
        # Actualizar el color del texto en el panel de productos
        scrollable_frame = None
        for widget in self.selected_frame.winfo_children():
            if isinstance(widget, ctk.CTkScrollableFrame):
                scrollable_frame = widget
                break
        if scrollable_frame:
            # Buscar en los contenedores de productos dentro del ScrollableFrame
            for product_container in scrollable_frame.winfo_children():
                if isinstance(product_container, ctk.CTkFrame):
                    # Buscar la etiqueta dentro del contenedor del producto
                    for label in product_container.winfo_children():
                        if isinstance(label, ctk.CTkLabel) and label.cget("text") == name:
                            label.configure(text_color=color)
                            return
      
    def get_yaw_from_quaternion(self, quaternion):
        # Método para convertir un cuaternión a ángulo de guiñada (yaw)
        # Utiliza la representación de cuaternión para obtener la orientación
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return yaw
      
    def spin_ros_node(self):
        # Método para hacer girar el nodo ROS
        # Mantiene activa la comunicación con ROS
        rclpy.spin(self.node)
    
    def handle_status(self, status, waypoint_name):
        # Método para manejar los cambios de estado de la navegación
        # Actualiza la interfaz según el estado actual con mensajes más amigables
        if status == "READY":
            self.status_label.configure(text="Ahora puede empezar a dirigirse a sus productos. Por favor dar clic a 'Siguiente producto'.")
            self.navigation_button.configure(state="enabled", text="Siguiente producto")
        elif status == "WAITING":
            self.status_label.configure(text="Ha llegado a su destino, cuando este listo dar clic a 'Siguiente producto'.")  
            self.navigation_button.configure(state="enabled")
        elif status == "NAVIGATING":
            if waypoint_name == "cashier":
                self.status_label.configure(text="SARA se está dirigiendo hacia: Caja Registradora")
            else:
                product_name = waypoint_name.replace("_", " ").title() if waypoint_name else "destino"
                self.status_label.configure(text=f"SARA se está dirigiendo hacia: {product_name}")
        elif status == "REACHED":
            # Mensaje más amigable cuando llega al destino
            product_name = waypoint_name.replace("_", " ").title() if waypoint_name else "destino"
            if waypoint_name == "cashier":
                self.status_label.configure(text="¡Ha llegado a la caja! Gracias por su compra.")
            else:
                self.status_label.configure(text=f"¡Ha llegado a {product_name}! Puede tomar su producto.")
        elif status == "FINISHED":
            self.status_label.configure(text="Ha finalizado su proceso de compra, cuando esté listo dar clic a a 'Ir a caja'") 
            self.navigation_button.configure(text="Ir a caja", state="enabled")
            self.go_to_cashier = True
        elif status == "SHOPPING_AGAIN":
            self.destroy()

    def handle_progress_bar(self, status, waypoint_name, completion_percentage):
        # Método para actualizar la barra de progreso
        # Muestra el avance de la navegación hacia el destino actual
        if completion_percentage:
            progress = float(completion_percentage) / 100
            self.progress_bar.set(progress)
        else:
            self.progress_bar.set(0)
            
        if status == "REACHED":
            self.progress_bar.set(1)
    

    def setup_signal_handlers(self):
        # Método para configurar los manejadores de señales del sistema
        # Permite una terminación limpia al recibir señales SIGINT o SIGTERM
        def signal_handler(signum, frame):
            if hasattr(self, 'is_closing') and self.is_closing:
                subprocess.run(['kill', '-9', str(os.getpid())], check=False)
            self.on_closing()
        
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

    def show_popup(self):
        # Método para mostrar una ventana emergente de opciones
        # Ofrece al usuario la opción de ir a la caja
        popup_window = ctk.CTkToplevel()
        popup_window.configure(fg_color=self.colors['primary_bg'])
        popup_window.title("Acciones")
        popup_window.geometry("300x150")
        
        label = ctk.CTkLabel(popup_window, text="Has terminado con tu compra, cuando estes listo presiona ir a caja", padx=20, pady=20)
        label.pack(expand=True)
        
        # Botón para ir a caja
        go_to_checkout_button = ctk.CTkButton(popup_window, text="Ir a Caja", command=self.go_to_checkout, fg_color=self.colors['button_bg'], text_color=self.colors['button_text'], hover_color=self.colors['button_hover'],)
        go_to_checkout_button.pack(side="left", padx=20, pady=10)
            
    def go_to_checkout(self):
        # Método para iniciar la navegación hacia la caja
        # Marca la posición de la caja y publica el mensaje correspondiente
        self.navigation_button.configure(state="disabled")  # Añadir esta línea
        self.add_cashier_marker()
        self.publish_cashier()
        self.cashier_reached = False
        self.wait_for_cashier_reached()

    def wait_for_cashier_reached(self):
        # Método para esperar a que el robot llegue a la caja
        # Verifica periódicamente si se ha alcanzado la posición de la caja
        if self.cashier_reached:
            self.finish_shopping()
        else:
            self.after(100, self.wait_for_cashier_reached)
       
    def finish_shopping(self):
        # Método para finalizar el proceso de compra
        # Muestra la ventana de agradecimiento y cierra la navegación
        if self.after_id is not None:
            self.after_cancel(self.after_id) 
        
        self.navigation_button.configure(state="disabled")  # Añadir esta línea
        self.new_window = ThanksWindow(self)
        self.withdraw()
        self.new_window.mainloop()
        
    def show_main_window(self):
        # Método para mostrar la ventana principal
        # Vuelve a la pantalla de selección de productos
        self.product_manager.deiconify()  # Show the ProductManager window
        self.destroy()  # Close the NavigationWindow
    
    def add_cashier_marker(self):
        # Método para añadir un marcador de la caja en el mapa
        # Crea o actualiza el marcador visual para la posición de la caja
        if self.cashier_marker:
            self.cashier_marker.remove()

        pixel_x = int((self.fixed_cash_location['x'] - self.origin[0]) / self.resolution)
        pixel_y = int((self.fixed_cash_location['y'] - self.origin[1]) / self.resolution)

        self.cashier_marker, = self.ax.plot(pixel_x, pixel_y, 'g*', markersize=15)
        #self.ax.legend()
        self.canvas.draw()
        
    def stop_ros_processes(self):
        # Método para detener todos los procesos ROS
        # Limpia los procesos y recursos relacionados con ROS
        print("Stopping ROS processes...")
        
        # Stop all launch processes
        for process in self.launch_processes:
            try:
                if process and process.poll() is None:
                    process.terminate()
                    process.wait(timeout=5)  # Wait up to 5 seconds for process to terminate
            except Exception as e:
                print(f"Error terminating process: {e}")
                try:
                    process.kill()  # Force kill if terminate fails
                except:
                    pass

        # Cleanup ROS node and executor
        if self.executor:
            try:
                self.executor.shutdown()
            except Exception as e:
                print(f"Error shutting down executor: {e}")

        if self.node:
            try:
                self.node.destroy_node()
            except Exception as e:
                print(f"Error destroying node: {e}")

        # Shutdown ROS context if it's still running
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception as e:
                print(f"Error shutting down ROS: {e}")

        self.node = None
        self.executor = None
        self.navigator = None
        self.odom_subscriber = None
        self.continue_nav_publisher = None
        self.launch_processes = []
   
    def show_info(self, message, title="Info"):
        # Método para mostrar ventanas de información al usuario
        # Crea una ventana emergente personalizada con un mensaje
        if self.current_popup is not None:
            self.current_popup.destroy()
        
        info_window = ctk.CTkToplevel()
        info_window.configure(fg_color=self.colors['primary_bg'])
        info_window.title(title)
        info_window.geometry("500x200")  # Ventana más grande
        
        # Centrar la ventana en la pantalla
        screen_width = info_window.winfo_screenwidth()
        screen_height = info_window.winfo_screenheight()
        x = (screen_width - 500) // 2  # 500 es el ancho de la ventana
        y = (screen_height - 200) // 2  # 200 es el alto de la ventana
        info_window.geometry(f"500x200+{x}+{y}")
        
        # Frame principal para organizar el contenido
        main_frame = ctk.CTkFrame(info_window, fg_color=self.colors['secondary_bg'])
        main_frame.pack(expand=True, fill="both", padx=20, pady=20)
        
        # Label con texto mejorado
        label = ctk.CTkLabel(
            main_frame, 
            text=message,
            font=('Arial', 16),  # Fuente más grande
            wraplength=400,  # Ajuste de texto
            justify="center"  # Centrar el texto
        )
        label.pack(expand=True, padx=20, pady=20)
        
        # Botón OK mejorado
        ok_button = ctk.CTkButton(
            main_frame,
            text="OK",
            command=info_window.destroy,
            width=100,
            height=35,
            font=('Arial', 14),
            fg_color=self.colors['button_bg'],
            text_color=self.colors['button_text'],
            hover_color=self.colors['button_hover']
        )
        ok_button.pack(pady=(0, 10))
        
        self.current_popup = info_window 
          
    def update_robot_position(self):
        # Método para actualizar la posición del robot en el mapa
        # Convierte las coordenadas del robot y actualiza su representación visual
        if not self.is_closing:
            try:
                if self.current_pose:
                    with self.lock:
                        # Convert map coordinates to pixel coordinates
                        pixel_x = int((self.current_pose['x'] - self.origin[0]) / self.resolution)
                        pixel_y = int((self.current_pose['y'] - self.origin[1]) / self.resolution)
                        orientation = self.current_pose.get('orientation', 0)  # in radians

                        # Remove the previous robot image if it exists
                        robot_width_pixels = self.robot_width / self.resolution
                        robot_length_pixels = self.robot_length / self.resolution
                        
                        # Update the position and rotation of the image
                        tr = mtransforms.Affine2D().rotate(orientation -math.pi/2).translate(pixel_x, pixel_y)
                        self.robot_imobj.set_transform(tr + self.ax.transData)
                        
                        # Update the extent of the image
                        extent = [-robot_width_pixels, robot_width_pixels,
                                -robot_length_pixels, robot_length_pixels]
                        self.robot_imobj.set_extent(extent)

                        if not self.is_closing:
                            self.canvas.draw()

                        if not self.pose_message_printed:  # Solo imprime si no se ha impreso antes
                            print("No current pose available")
                            self.pose_message_printed = True

                if not self.is_closing:
                    self.after(100, self.update_robot_position)

            except Exception as e:
                print(f"Error in update_robot_position: {e}")
                if not self.is_closing:
                    self.after(100, self.update_robot_position)

    def actualizar_reloj_y_fecha(self):
        # Método para actualizar la hora y fecha mostradas
        # Actualiza las etiquetas de reloj y fecha cada segundo
        now = datetime.datetime.now()
        self.label_reloj.configure(text=now.strftime("%H:%M:%S"))
        self.label_fecha.configure(text=now.strftime("%Y-%m-%d"))
        self.after(1000, self.actualizar_reloj_y_fecha)
  
    def create_plot(self, frame):
        # Método para crear el gráfico del mapa
        # Configura la visualización del mapa y añade la imagen del robot
        self.fig, self.ax = plt.subplots(figsize=(6, 4), dpi=100)
        # Ocultar los ejes y números
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        # Opcional: también puedes ocultar los bordes del gráfico
        self.ax.spines['top'].set_visible(False)
        self.ax.spines['right'].set_visible(False)
        self.ax.spines['bottom'].set_visible(False)
        self.ax.spines['left'].set_visible(False)
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=frame)
        self.canvas.get_tk_widget().pack(fill=ctk.BOTH, expand=True)

        self.update_map_plot()
        
        # Cargar y configurar la imagen del robot
        pkg_dir = get_package_share_directory('turtlemart')
        robot_image_path = os.path.join(pkg_dir, 'images/shoppingcart.png')
        self.robot_image = plt.imread(robot_image_path)
        robot_width_data = self.robot_width / self.resolution
        robot_height_data = self.robot_length / self.resolution
        
        # Create the image object on the axes, swapping width and height in extent
        self.robot_imobj = self.ax.imshow(self.robot_image, 
                                          extent=[-robot_height_data/2, robot_height_data/2, 
                                                  -robot_width_data/2, robot_width_data/2],
                                          zorder=2,  # Ensure the robot is drawn on top of the map
                                          alpha=0.8)  # Slight transparency to see the map underneath

        # Set the initial position off-screen
        self.robot_imobj.set_transform(mtransforms.Affine2D().translate(-1000, -1000) + self.ax.transData)
      
    def update_map_plot(self):
        # Método para actualizar el gráfico del mapa
        # Carga el mapa y muestra las ubicaciones de los productos
        self.ax.clear()
        # Cargar el mapa y actualizar el gráfico
        self.map_array, self.resolution, self.origin = self.load_map()
        self.ax.imshow(np.flipud(self.map_array), cmap='gray', origin='lower')
        map_height, map_width = self.map_array.shape
        self.ax.set_xlim(0, map_width)
        self.ax.set_ylim(0, map_height)
        # Obtener y plotear ubicaciones de productos
        self.plot_product_locations()
        
    def load_map(self):
        # Método para cargar el mapa desde un archivo YAML
        # Lee el archivo de configuración del mapa y carga la imagen correspondiente
        bringup_dir = get_package_share_directory('turtlemart')
        if self.nav_mode == "Real":
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

    def plot_product_locations(self):
        # Método para mostrar las ubicaciones de los productos en el mapa
        # Dibuja marcadores en las posiciones de los productos seleccionados
        # Obtener ubicaciones de la base de datos
        locations = self.get_product_locations()

        for loc in locations:
            # Convertir las coordenadas de mapa a píxeles
            pixel_x = int((loc['x'] - self.origin[0]) / self.resolution)
            pixel_y = int((loc['y'] - self.origin[1]) / self.resolution)
            # Plottear las ubicaciones en el mapa
            self.ax.plot(pixel_x, pixel_y, 'ro')  # 'ro' indica puntos rojos

        self.canvas.draw()

    def get_product_locations(self):
        # Método para obtener las ubicaciones de los productos de la base de datos
        # Consulta la base de datos para obtener nombres y coordenadas
        db_dir = get_source_db_path('turtlemart', 'products.db')
        conn = sqlite3.connect(db_dir)
        cursor = conn.cursor()
        cursor.execute('SELECT name, x, y FROM selected_products')
        locations = [{'name': row[0], 'x': row[1], 'y': row[2]} for row in cursor.fetchall()]
        conn.close()
        return locations

    def handle_navigation_button(self):
        # Método para manejar la acción del botón de navegación
        # Implementa la lógica de diferentes estados del botón
        if not self.calibration_complete:
            # Primera fase: iniciar calibración
            if not self.launch_thread or not self.launch_thread.is_alive():
                self.launch_thread = self.start_thread(self.launch_and_update)
                self.navigation_button.configure(state="disabled")
                print("Launching ROS 2 files in background...")
        else:
            # Segunda fase: navegación
            if not self.navigation_started:
                print("Iniciando navegación...")
                self.start_thread(self.run_navigation)
                self.navigation_started = True
                self.navigation_button.configure(state="disabled")
            elif self.go_to_cashier:
                self.navigation_button.configure(state="disabled")
                self.go_to_checkout()
            else:
                print("La navegación ya ha comenzado. Ejecutando otra acción...")
                self.perform_alternate_action()

    def launch_and_update(self):
        # Método para lanzar archivos ROS2 y actualizar la interfaz
        # Inicia los procesos necesarios y actualiza el estado del botón
        # Lanzar los archivos ROS2
        self.launch_ros2_files()
        # Esperar un tiempo prudencial para que todo se inicie
        time.sleep(10)  # Ajusta este tiempo según sea necesario
        # Actualizar el botón
        self.calibration_complete = True
        def update_ui():
            self.navigation_button.configure(
                text="Buscar productos",
                state="normal"
            )
            self.status_label.configure(
                text=" Haga clic en 'Buscar productos' para encontrar la ruta mas eficiente."
            )
        self.after(0, update_ui)
        
                
    def launch_ros2_files(self):
        # Método para lanzar los archivos de configuración ROS2
        # Inicia los procesos necesarios según el modo de navegación
        launch_commands = [
            "ros2 launch turtlemart mux.launch.py",
        ]

        if self.nav_mode == "Real":
            launch_commands.append("ros2 launch turtlemart real_nav.launch.py")
        else:
            launch_commands.append("ros2 launch turtlemart navagv.launch.py")
        
        devnull = open(os.devnull, 'w')
        for cmd in launch_commands:
            process = subprocess.Popen(
                cmd, 
                shell=True, 
                stdout=devnull, 
                stderr=subprocess.STDOUT
            )
            self.launch_processes.append(process)
        
        print("Launch files started successfully (output suppressed).")
    
    def launch_basic_control(self):
        # Método para lanzar el control básico del robot
        # Inicia y monitorea el proceso de control básico
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
        
    def is_process_running(self, process_name):
        # Método para verificar si un proceso está en ejecución
        # Comprueba si existe un proceso con el nombre especificado
        """Verifica si un proceso con el nombre especificado ya está en ejecución"""
        try:
            # `pgrep` busca procesos que coincidan con el nombre
            output = subprocess.check_output(["pgrep", "-f", process_name])
            return bool(output.strip())  # Si hay salida, el proceso está corriendo
        except subprocess.CalledProcessError:
            return False  # Si no se encuentra el proceso, retorna False

    def run_navigation(self):
        # Método para ejecutar la navegación
        # Crea una instancia del navegador apropiado y ejecuta la navegación
        try:
            navigator_class = RealNavigator if self.nav_mode == "Real" else SimNavigator
            navigator = navigator_class()
            waypoints_reached = navigator.navigate()
            print("Navegación completa")
            if not self.is_closing:
                self.publish_continue_nav()
                # Use after_idle for thread-safe GUI updates
                self.after_idle(lambda: self.navigation_button.configure(state="normal"))

        except rclpy.executors.ExternalShutdownException:
            print("ROS shutdown during navigation")
        except Exception as e:
            print(f"Error en la navegación: {e}")
            if not self.is_closing:
                self.after_idle(lambda: self.navigation_button.configure(state="normal"))
            
    def __del__(self):
        # Método destructor de la clase
        # Limpia los recursos cuando se destruye la instancia
        # Terminate launch file processes when the window is closed
        for process in self.launch_processes:
            process.terminate()
        
        if self.launch_thread and self.launch_thread.is_alive():
            self.launch_thread.join(timeout=1)  # Dar tiempo para que termine
            
        if self.node:
            self.node.destroy_node()
        rclpy.shutdown()

    def perform_alternate_action(self):
        # Método para ejecutar acciones alternativas
        # Publica mensajes para continuar la navegación
        self.navigation_button.configure(state="disabled")
        self.publish_continue_nav()
        self.continue_nav_published = True
        self.publish_stop()

    def publish_continue_nav(self):
        # Método para publicar el mensaje de continuar la navegación
        # Envía un mensaje al tópico correspondiente
        msg = String()
        msg.data = "continue"
        self.continue_nav_publisher.publish(msg)
        
    def publish_stop(self):
        # Método para publicar el mensaje de detener la navegación
        # Envía un mensaje de parada al tópico correspondiente
        msg = String()
        msg.data = "stop"
        self.continue_nav_publisher.publish(msg)
        
    def publish_cashier(self):
        # Método para publicar el mensaje de ir a la caja
        # Envía un mensaje para dirigirse a la caja
        msg = String()
        msg.data = "cash"
        self.cashier_publisher.publish(msg)

    def publish_shop_again(self):
        # Método para publicar el mensaje de volver a comprar
        # Envía un mensaje para reiniciar el proceso de compra
        msg = String()
        msg.data = "shop_again"
        self.cashier_publisher.publish(msg)
    
    def cleanup_gui(self):
        # Método para limpiar los recursos de la interfaz gráfica
        # Cancela callbacks pendientes y cierra ventanas
        """Limpia todos los recursos relacionados con la GUI"""
        print("Cleaning up GUI resources...")
        try:
            # Cancelar callbacks pendientes
            for after_id in self.tk.eval('after info').split():
                try:
                    self.after_cancel(after_id)
                except Exception as e:
                    print(f"Error canceling after callback {after_id}: {e}")
            
            # Cerrar popups
            if hasattr(self, 'current_popup') and self.current_popup:
                try:
                    self.current_popup.destroy()
                except Exception as e:
                    print(f"Error closing popup: {e}")
            
            # Limpiar matplotlib
            if hasattr(self, 'fig'):
                try:
                    plt.close(self.fig)
                except Exception as e:
                    print(f"Error closing matplotlib figure: {e}")
                
            # Limpiar canvas
            if hasattr(self, 'canvas'):
                try:
                    self.canvas.get_tk_widget().destroy()
                except Exception as e:
                    print(f"Error destroying canvas: {e}")
        except Exception as e:
            print(f"Error during GUI cleanup: {e}")

    def cleanup_threads(self):
        # Método para limpiar los hilos creados
        # Espera a que los hilos terminen de forma ordenada
        """Limpia todos los hilos"""
        print("Cleaning up threads...")
        if hasattr(self, 'threads'):
            for thread in self.threads:
                try:
                    if thread and thread.is_alive():
                        thread.join(timeout=2.0)
                except Exception as e:
                    print(f"Error joining thread: {e}")

    def cleanup_processes(self):
        # Método para limpiar procesos externos
        # Termina los procesos relacionados con ROS y otros subprocesos
        """Limpia todos los procesos relacionados con ROS y otros subprocesos"""
        print("Cleaning up processes...")
        
        ros_processes = [
            "rviz2", "gazebo", "gzclient", "gzserver",
            "ros2", "map_server", "amcl", "nav2",
            "robot_state_publisher", "joint_state_publisher"
        ]
        
        # Intento de terminación suave
        for proc_name in ros_processes:
            try:
                subprocess.run(['pkill', '-TERM', proc_name], check=False)
            except Exception as e:
                print(f"Error terminating {proc_name}: {e}")
        
        time.sleep(1)
        
        # Forzar terminación
        for proc_name in ros_processes:
            try:
                subprocess.run(['pkill', '-9', proc_name], check=False)
            except Exception as e:
                print(f"Error force killing {proc_name}: {e}")
        
        # Terminar procesos lanzados
        for process in self.launch_processes:
            try:
                if process and process.poll() is None:
                    process.terminate()
                    process.wait(timeout=2)
                    if process.poll() is None:
                        process.kill()
            except Exception as e:
                print(f"Error killing launch process: {e}")

    def cleanup_ros(self):
        # Método para limpiar recursos de ROS
        # Detiene la navegación y limpia nodos, publishers y subscribers
        """Limpia todos los recursos relacionados con ROS"""
        print("Cleaning up ROS resources...")
        try:
            # Intentar detener la navegación primero
            if hasattr(self, 'navigator') and self.navigator:
                try:
                    self.navigator.cancelNavigation()
                except:
                    pass

            # Limpiar publishers y subscribers
            ros_components = [
                'continue_nav_publisher', 'cashier_publisher', 
                'odom_subscriber', 'is_joy_on_subscriber', 
                'status_subscriber'
            ]
            
            for component in ros_components:
                if hasattr(self, component):
                    try:
                        component_obj = getattr(self, component)
                        if component_obj:
                            component_obj.destroy()
                    except:
                        pass

            # Limpiar executor y node
            if hasattr(self, 'executor') and self.executor:
                try:
                    self.executor.shutdown()
                except:
                    pass
                    
            if hasattr(self, 'node') and self.node:
                try:
                    self.node.destroy_node()
                except:
                    pass

            if rclpy.ok() and self.ros_initialized:
                try:
                    rclpy.shutdown()
                except:
                    pass
                    
        except Exception as e:
            print(f"Error during ROS cleanup: {e}")

    def on_closing(self):
        # Método para manejar el cierre de la aplicación
        # Implementa un proceso completo de limpieza de recursos
        """Manejador principal de cierre"""
        if hasattr(self, 'is_closing') and self.is_closing:
            print("DEBUG: Already in closing process, forcing immediate kill...")
            subprocess.run(['kill', '-9', str(os.getpid())], check=False)
            return

        print("\nDEBUG: Starting cleanup process...")
        self.is_closing = True
        
        try:
            # Script de limpieza más exhaustivo
            kill_script = f"""#!/bin/bash
            
            # Función para matar procesos y sus hijos recursivamente
            kill_process_tree() {{
                local parent=$1
                local children=$(ps -o pid --no-headers --ppid "$parent")
                
                for child in $children; do
                    kill_process_tree "$child"
                done
                
                kill -9 "$parent" 2>/dev/null
            }}
            
            # Primero intentar detener la navegación y procesos relacionados
            nav2_pids=$(pgrep -f "nav2")
            for pid in $nav2_pids; do
                kill_process_tree "$pid"
            done
            
            # Lista exhaustiva de procesos a matar
            PROCESS_PATTERNS=(
                # Procesos de navegación
                "bt_navigator"
                "controller_server"
                "planner_server"
                "recoveries_server"
                "waypoint_follower"
                "lifecycle_manager"
                "navigation2"
                "amcl"
                "nav2"
                # Procesos de visualización
                "rviz"
                "rviz2"
                # Procesos de simulación
                "gazebo"
                "gzclient"
                "gzserver"
                # Procesos del robot
                "robot_state_publisher"
                "joint_state_publisher"
                "transforms"
                "turtlebot3"
                "slam_toolbox"
                # Procesos ROS2 generales
                "ros2"
                "_ros2"
                "ros2 launch"
                "ros2 run"
                "launch.py"
                # Procesos Python relacionados
                "python3.*ros2"
                # Procesos del mapa
                "map_server"
                "map_saver"
                # Procesos de control
                "basic_control"
                "mux"
                # Nodos y servicios
                "transform_listener_impl"
                "parameter_server"
                "local_costmap"
                "global_costmap"
            )

            # Matar cada proceso y sus hijos
            for pattern in "${{PROCESS_PATTERNS[@]}}"; do
                echo "Killing processes matching: $pattern"
                pids=$(pgrep -f "$pattern")
                for pid in $pids; do
                    kill_process_tree "$pid"
                done
                pkill -9 -f "$pattern"
            done
            
            # Asegurarse de que los procesos principales estén muertos
            pkill -9 -f "gazebo"
            pkill -9 -f "rviz"
            pkill -9 -f "nav2"
            pkill -9 -f "python3.*ros2"
            
            # Detener el daemon de ROS2
            ros2 daemon stop
            
            # Desbloquear terminal
            pkill -CONT -f "bash"
            
            # Matar este proceso al final
            kill -9 {os.getpid()}
            """
            
            # Intentar detener la navegación desde Python primero
            if hasattr(self, 'navigator') and self.navigator:
                try:
                    self.navigator.cancelNavigation()
                except:
                    pass

            # Si hay un publisher de navegación, intentar enviar señal de parada
            if hasattr(self, 'continue_nav_publisher'):
                try:
                    msg = String()
                    msg.data = "stop"
                    self.continue_nav_publisher.publish(msg)
                except:
                    pass
            
            # Crear y ejecutar script de limpieza
            with tempfile.NamedTemporaryFile(mode='w', suffix='.sh', delete=False) as f:
                f.write(kill_script)
                script_path = f.name
            
            os.chmod(script_path, 0o755)
            
            subprocess.Popen(['bash', script_path],
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL,
                            start_new_session=True)
            
            subprocess.Popen(['rm', script_path],
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)
                            
            print("DEBUG: Cleanup script launched, forcing exit...")
            
        except Exception as e:
            print(f"DEBUG: Error in cleanup - {str(e)}")
        
        # Forzar terminación
        subprocess.run(['kill', '-9', str(os.getpid())], check=False)

    def kill_all_ros_processes(self):
        # Método para matar todos los procesos ROS inmediatamente
        # Utiliza pkill para terminar procesos por nombre
        """Mata todos los procesos de ROS inmediatamente"""
        try:
            # Primero matar los procesos específicos
            processes_to_kill = [
                "rviz2",
                "gazebo",
                "gzclient",
                "gzserver",
                "map_server",
                "amcl",
                "nav2",
                "robot_state_publisher",
                "joint_state_publisher",
                "turtlebot3",
                "slam_toolbox",
                "navigation2",
                "bt_navigator",
                "controller_server",
                "planner_server",
                "behavior_server",
                "lifecycle_manager"
            ]
            
            # Matar procesos conocidos
            for proc_name in processes_to_kill:
                subprocess.run(['pkill', '-9', '-f', proc_name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            
            # Matar todos los procesos de ros2
            subprocess.run(['pkill', '-9', '-f', 'ros2'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            
            # Matar procesos de launch
            if hasattr(self, 'launch_processes'):
                for process in self.launch_processes:
                    try:
                        if process and process.poll() is None:
                            process.kill()
                    except:
                        pass
                        
            # Esperar un momento para asegurar que los procesos mueran
            time.sleep(0.1)
            
        except Exception as e:
            print(f"Error killing ROS processes: {e}")

    def stop_navigation(self):
        # Método para detener la navegación en curso
        # Cancela la navegación y publica mensaje de parada

        try:
            if hasattr(self, 'navigator') and self.navigator:
                self.navigator.cancelNavigation()
            
            # Publicar mensaje de parada si es necesario
            if hasattr(self, 'continue_nav_publisher'):
                msg = String()
                msg.data = "stop"
                self.continue_nav_publisher.publish(msg)
        except Exception as e:
            print(f"Error stopping navigation: {e}")

    def force_kill_ros_processes(self):
        # Método para forzar la terminación de todos los procesos ROS
        # Último recurso para matar procesos que no responden

        ros_processes = [
            "rviz2", "gazebo", "gzclient", "gzserver",
            "ros2", "map_server", "amcl", "nav2",
            "robot_state_publisher", "joint_state_publisher",
            "python3"  # Cuidado: esto matará todos los procesos Python
        ]
        
        for proc_name in ros_processes:
            try:
                # Forzar kill inmediatamente
                subprocess.run(['pkill', '-9', proc_name], check=False)
            except Exception as e:
                print(f"Error force killing {proc_name}: {e}")

    def start_thread(self, target, *args, **kwargs):
        # Método auxiliar para iniciar y rastrear hilos
        # Crea un hilo en modo daemon y lo almacena para su posterior limpieza
        thread = threading.Thread(target=target, args=args, kwargs=kwargs)
        thread.daemon = True
        self.threads.append(thread)
        thread.start()
        return thread

    def view_selected_products(self):
        # Método para mostrar los productos seleccionados
        # Recrea la lista de productos seleccionados con sus colores actuales
        self.selected_frame.destroy()
        self.selected_frame = ctk.CTkFrame(self.right_frame, width=250, fg_color=self.colors['list_bg'])
        self.selected_frame.pack(side=ctk.TOP, fill=ctk.BOTH, expand=True, padx=10, pady=10)

        # Título con fuente constante
        title_label = ctk.CTkLabel(
            self.selected_frame, 
            text="Lista de productos",
            text_color=self.colors['text_primary'],
            font=self.PRODUCT_TITLE_FONT
        )
        title_label.pack(pady=(10, 15))
        db_dir = get_source_db_path('turtlemart', 'products.db')
        conn = sqlite3.connect(db_dir)
        cursor = conn.cursor()
        cursor.execute('SELECT name FROM selected_products')
        rows = cursor.fetchall()
        conn.close()

        products_container = ctk.CTkScrollableFrame(
            self.selected_frame,
            fg_color="transparent",
            height=400
        )
        products_container.pack(fill=ctk.BOTH, expand=True, padx=5, pady=5)

        # Limpiar el diccionario de labels antiguos
        self.product_labels.clear()

        for row in rows:
            product_name = row[0]
            product_container = ctk.CTkFrame(products_container, fg_color=self.colors['list_bg'], height=50)
            product_container.pack(fill=ctk.X, pady=10, padx=15)
            product_container.pack_propagate(False)

            # Usar el color guardado en el diccionario si existe, si no usar el color por defecto
            text_color = self.product_colors.get(product_name, self.colors['text_primary'])
            
            label = ctk.CTkLabel(
                product_container, 
                text=product_name,
                text_color=text_color,
                font=self.PRODUCT_FONT
            )
            label.pack(expand=True)
            self.product_labels[product_name] = label

    def refresh_product_list(self):
        # Método para actualizar la lista de productos
        # Actualiza la visualización de los productos seleccionados
        # Llamar a view_selected_products para actualizar la lista completa
        self.view_selected_products()

def get_source_db_path(package_name, db_filename):
    # Función para obtener la ruta de la base de datos en el directorio src del paquete
    # Navega desde el directorio share hasta la ubicación de la base de datos
    # siguiendo la estructura estándar de un workspace ROS2

    # Obtener el directorio share del paquete
    share_dir = get_package_share_directory(package_name)
    
    # Navegar hasta la raíz del workspace (subir 4 niveles: share/package/install/workspace)
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(share_dir))))
    
    # Construir la ruta a la base de datos en src
    db_path = os.path.join(workspace_root, 'src', package_name, 'database', db_filename)
    
    #print(f"Trying to access database at: {db_path}")
    
    return db_path

if __name__ == "__main__":
    # Punto de entrada principal del programa
    # Crea y ejecuta la ventana de navegación en modo real
    app = RealNavWindow()
    app.mainloop()