#!/usr/bin/env python3
# Autor: David Capacho Parra
# Fecha: Febrero 2025
# Descripción: Ventana de agradecimiento del sistema Smart Autonomous Retail Assistant (SARA)
# Implementa la pantalla final que se muestra al finalizar el proceso de compra,
# agradeciendo al usuario y solicitando su valoración de la experiencia.

import customtkinter as ctk
import datetime
import os
import signal
import subprocess
import tempfile
import threading
import time
from std_msgs.msg import String

class ThanksWindow(ctk.CTk):
    # Clase que implementa la ventana de agradecimiento final
    # Muestra un mensaje de agradecimiento y opciones para evaluar la experiencia
    def __init__(self, master=None):
        # Inicialización de la ventana de agradecimiento
        # Configura la interfaz gráfica y establece los manejadores de señales
        super().__init__()
        self.master = master
        self.after_id = None
        
        # Definición de los colores estandarizados
        # Establece la paleta de colores para mantener la consistencia visual
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
        
        # Configuración de la interfaz
        self.setup_ui()
        signal.signal(signal.SIGINT, self.signal_handler)

        self.is_closing = False
        self.setup_signal_handlers()
        self.protocol("WM_DELETE_WINDOW", self.on_closing)

    def setup_ui(self):
        # Método para configurar la interfaz de usuario
        # Crea y dispone todos los elementos visuales de la interfaz
        self.title("Smart Autonomous Retail Assistant")
        self.geometry("%dx%d+0+0" % (self.winfo_screenwidth(), self.winfo_screenheight()))
        self.resizable(width=1, height=1)

        # Frame en la parte superior
        # Contiene el título y el reloj/fecha
        top_frame = ctk.CTkFrame(self, height=100, fg_color=self.colors['frame_bg'])
        top_frame.pack(side=ctk.TOP, fill=ctk.X)

        # Configurar grid para la disposición
        top_frame.grid_columnconfigure(0, weight=1)

        # Título centrado
        title_label = ctk.CTkLabel(
            top_frame,
            text="¡Hemos llegado a la caja!",
            font=("Helvetica", 35, "bold"),
            text_color=self.colors['text_primary'],
            justify="center"
        )
        title_label.grid(row=0, column=0, padx=20, pady=10, sticky="nsew")

        # Contenedor para fecha y hora en esquina superior derecha
        clock_container = ctk.CTkFrame(top_frame, fg_color=self.colors['frame_bg'])
        clock_container.grid(row=0, column=1, padx=20, pady=5, sticky="e")

        # Reloj
        self.label_reloj = ctk.CTkLabel(
            clock_container,
            font=('ARIAL', 25, 'bold'),
            text_color=self.colors['text_secondary']
        )
        self.label_reloj.pack(side=ctk.TOP, pady=2)

        # Fecha
        self.label_fecha = ctk.CTkLabel(
            clock_container,
            font=('ARIAL', 25, 'bold'),
            text_color=self.colors['text_secondary']
        )
        self.label_fecha.pack(side=ctk.TOP, pady=2)
        self.actualizar_reloj_y_fecha()

        # Frame principal central
        # Contiene el mensaje de agradecimiento y opciones de valoración
        main_frame = ctk.CTkFrame(self, fg_color=self.colors['primary_bg'])
        main_frame.pack(side=ctk.LEFT, fill=ctk.BOTH, padx=20, pady=20, expand=True)

        thank_you_label = ctk.CTkLabel(
            main_frame,
            text="\n\n"
            "Gracias por confiar en SARA.\n\n"
            "¿Qué te pareció tu experiencia?\n\n"
            "[😊] [😐] [😞]\n\n"
            "No olvides retirar todos tus productos del carrito.\n\n"
            "¡Vuelve pronto!",
            font=('Helvetica', 35, 'bold'),
            text_color=self.colors['text_primary'],
            wraplength=500,
        )
        thank_you_label.pack(expand=True, padx=20, pady=20, anchor='center')

    def actualizar_reloj_y_fecha(self):
        # Método para actualizar la hora y fecha mostradas
        # Actualiza las etiquetas de reloj y fecha cada segundo
        now = datetime.datetime.now()
        self.label_reloj.configure(text=now.strftime("%H:%M:%S"))
        self.label_fecha.configure(text=now.strftime("%Y-%m-%d"))
        self.after_id = self.after(1000, self.actualizar_reloj_y_fecha)

    def signal_handler(self, sig, frame):
        # Manejador de señales para interrupciones del sistema
        # Permite detener la aplicación correctamente ante la señal SIGINT (Ctrl+C)
        print("Ctrl+C detectado, cerrando la aplicación...")
        self.on_closing()

    def on_closing(self):
        # Método para manejar el cierre normal de la ventana
        # Cancela los temporizadores y destruye la ventana
        print("Cerrando la ventana correctamente...")
        if self.after_id is not None:
            self.after_cancel(self.after_id)
        self.quit()
        self.destroy()

    def setup_signal_handlers(self):
        # Método para configurar los manejadores de señales del sistema
        # Permite una terminación limpia al recibir señales SIGINT o SIGTERM
        """Configura los manejadores de señales para SIGINT y SIGTERM"""
        def signal_handler(signum, frame):
            if hasattr(self, 'is_closing') and self.is_closing:
                subprocess.run(['kill', '-9', str(os.getpid())], check=False)
            self.on_closing()
        
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

    def on_closing(self):
        # Método para manejar el cierre completo de la aplicación
        # Implementa un proceso exhaustivo de limpieza de recursos
        """Manejador principal de cierre mejorado"""
        if hasattr(self, 'is_closing') and self.is_closing:
            print("DEBUG: Already in closing process, forcing immediate kill...")
            subprocess.run(['kill', '-9', str(os.getpid())], check=False)
            return

        print("\nDEBUG: Starting cleanup process...")
        self.is_closing = True
        
        try:
            # Script de limpieza exhaustivo
            # Crea un script bash para terminar todos los procesos relacionados
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
                # Procesos adicionales de ROS2
                "rosmaster"
                "roscore"
                "rosout"
                "rosbag"
                "ros2_lifecycle"
                # Procesos de SLAM y mapeo
                "cartographer"
                "slam_gmapping"
                "rtabmap"
                # Procesos de navegación adicionales
                "move_base"
                "dwa_local_planner"
                "teb_local_planner"
                "costmap_2d"
                # Procesos de visualización adicionales
                "rqt"
                "plotjuggler"
                # Procesos de transformación
                "static_transform_publisher"
                "tf2_ros"
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

if __name__ == '__main__':
    # Punto de entrada principal del programa
    # Crea y ejecuta la ventana de agradecimiento
    app = ThanksWindow()
    app.mainloop()