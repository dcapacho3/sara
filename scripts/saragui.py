#!/usr/bin/env python3
# Autor: David Capacho Parra
# Fecha: Febrero 2025
# Descripción: Interfaz principal del sistema Smart Autonomous Retail Assistant (SARA)
# Implementa una interfaz gráfica de usuario para la selección de productos
# y navegación en el entorno del retail, permitiendo al usuario interactuar
# con el sistema de asistencia autónoma para compras.

import sqlite3
import customtkinter as ctk
from tkinter import simpledialog
import os
import datetime
import signal
from base_navgui import UnifiedNavigationWindow
from PIL import Image
from ament_index_python.packages import get_package_share_directory

def get_source_db_path(package_name, db_filename):
    """
    # Función para localizar la base de datos en el directorio src del paquete
    # Navega desde el directorio share hasta la ubicación de la base de datos
    # siguiendo la estructura estándar de un workspace ROS2
    """
    # Obtener el directorio share del paquete
    share_dir = get_package_share_directory(package_name)
    
    # Navegar hasta la raíz del workspace (subir 4 niveles: share/package/install/workspace)
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(share_dir))))
    
    # Construir la ruta a la base de datos en src
    db_path = os.path.join(workspace_root, 'src', package_name, 'database', db_filename)
    
    #print(f"Trying to access database at: {db_path}")
    
    return db_path

class CTkVirtualKeyboard(ctk.CTkToplevel):
    # Clase que implementa un teclado virtual para la interfaz táctil
    # Permite la entrada de texto sin necesidad de un teclado físico
    # Facilita la interacción con el sistema en entornos de kiosco
    def __init__(self, parent, entry_widget, colors):
        # Inicialización del teclado virtual
        # Configura la ventana emergente y los controles del teclado
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
        # Método para crear la zona de visualización del texto
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
        # Método para crear la disposición de teclas del teclado virtual
        # Define el layout de las teclas y crea los botones correspondientes
        # Layout del teclado
        layouts = {
            'default': [
                ['1', '2', '3', '4', '5', '6', '7', '8', '9', '0'],
                ['Q', 'W', 'E', 'R', 'T', 'Y', 'U', 'I', 'O', 'P'],
                ['A', 'S', 'D', 'F', 'G', 'H', 'J', 'K', 'L', 'Ñ'],
                ['Z', 'X', 'C', 'V', 'B', 'N', 'M', '.', '-', '_']
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
        # Implementa la funcionalidad de la tecla de retroceso
        current_pos = self.display.index(ctk.INSERT)
        if current_pos > 0:
            self.display.delete(current_pos - 1)
            
    def toggle_layout(self):
        # Método para cambiar entre diferentes disposiciones de teclado
        # Actualmente implementado como un método vacío para futura expansión
        pass
        
    def accept(self):
        # Método para aceptar el texto ingresado
        # Transfiere el texto del teclado virtual al campo de entrada original
        text = self.display.get()
        self.entry_widget.delete(0, 'end')
        self.entry_widget.insert(0, text)
        self.destroy()
        
    def cancel(self):
        # Método para cancelar la entrada de texto
        # Cierra el teclado virtual sin modificar el campo de entrada original
        self.destroy()

# Función auxiliar para mostrar el teclado virtual
def show_virtual_keyboard(entry_widget, parent, colors):
    # Función que crea y muestra una instancia del teclado virtual
    # Espera hasta que la ventana del teclado se cierre
    keyboard = CTkVirtualKeyboard(parent, entry_widget, colors)
    parent.wait_window(keyboard)
        

class ProductManager:
    # Clase principal que gestiona la interfaz de selección de productos
    # Coordina la visualización, búsqueda y selección de productos
    # así como la navegación hacia estos productos
    def __init__(self, root):
        # Inicialización del gestor de productos
        # Configura variables de estado y elementos de la interfaz
        self.show_mode_selector = True 
        #self.show_mode_selector = False 
        self.root = root
        self.checkbox_vars = {}
        self.after_id = None
        self.navigation_window = None
        self.is_closing = False
        self.db_connection = None

        # Definición de la paleta de colores
        # Establece los colores para todos los elementos de la interfaz
        # organizados por categorías para facilitar la consistencia visual
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
        signal.signal(signal.SIGTERM, self.signal_handler)  # Agregar esta línea
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def setup_ui(self):
        # Método para configurar la interfaz de usuario
        # Crea y dispone todos los elementos visuales de la interfaz
        ctk.set_appearance_mode("light")
        self.root.configure(fg_color=self.colors['primary_bg'])
        self.root.title("Smart Autonomous Retail Assistant")
        self.root.geometry("%dx%d+0+0" % (self.root.winfo_screenwidth(), self.root.winfo_screenheight()))
        self.root.resizable(width=1, height=1)

        # Frame izquierdo para imagen y controles
        # Contiene el logo, selector de modo y texto SARA
        left_frame = ctk.CTkFrame(self.root, width=400, fg_color=self.colors['secondary_bg'])
        left_frame.pack(side=ctk.LEFT, fill=ctk.Y)
        left_frame.pack_propagate(False)

        # Frame derecho contenedor
        # Aloja el contenido principal de la aplicación
        right_container = ctk.CTkFrame(self.root, fg_color=self.colors['primary_bg'])
        right_container.pack(side=ctk.LEFT, fill=ctk.BOTH, expand=True)

        # Frame superior con reloj/fecha
        # Muestra la hora y fecha actuales en la parte superior
        top_frame = ctk.CTkFrame(right_container, height=100, fg_color=self.colors['secondary_bg'])
        top_frame.pack(fill=ctk.X, padx=10, pady=10)

        # Frame para reloj y fecha
        # Agrupa los elementos de fecha y hora
        clock_frame = ctk.CTkFrame(top_frame, fg_color=self.colors['secondary_bg'])
        clock_frame.pack(side=ctk.RIGHT, padx=20)
        
        now = datetime.datetime.now()

        self.label_fecha = ctk.CTkLabel(
            clock_frame, 
            text=now.strftime("%Y-%m-%d"),
            font=('ARIAL', 25, 'bold'),
            text_color=self.colors['text_primary']
        )
        self.label_fecha.pack(side=ctk.TOP, pady=5)
        
        self.label_reloj = ctk.CTkLabel(
            clock_frame, 
            text=now.strftime("%H:%M:%S"),
            font=('ARIAL', 25, 'bold'),
            text_color=self.colors['text_primary']
        )
        self.label_reloj.pack(side=ctk.TOP, pady=5)

        # Imagen de usuario
        # Carga y muestra el logo del usuario en el panel izquierdo
        pkg_dir = get_package_share_directory('turtlemart')
        image_path = os.path.join(pkg_dir, 'images/userlogo.png')
        img = Image.open(image_path)
        size = 300

        image_frame = ctk.CTkFrame(left_frame, width=size, height=size, fg_color=self.colors['secondary_bg'])
        image_frame.pack(padx=50, pady=30)
        image_frame.pack_propagate(False)  # Mantener el tamaño del frame

        img = img.resize((size, size), Image.LANCZOS)
        
        ctk_image = ctk.CTkImage(light_image=img, size=(size, size))
        label = ctk.CTkLabel(image_frame, image=ctk_image, text="")
        label.pack(expand=True, fill="both")

        # Configuración del selector de modo 
        # Permite elegir entre modo real y simulación
        option_frame = ctk.CTkFrame(left_frame, fg_color=self.colors['secondary_bg'])
        option_frame.pack(expand=True, fill=ctk.BOTH, padx=20, pady=5)

        option_center_frame = ctk.CTkFrame(option_frame, fg_color=self.colors['secondary_bg'])
        option_center_frame.pack(expand=True)

        welcome_label = ctk.CTkLabel(
            option_center_frame, 
            text="¡Bienvenido!", 
            font=("Arial", 36, "italic", "bold"),
            text_color=self.colors['text_primary']
        )
        welcome_label.pack(pady=(0, 30))

        self.navigation_mode = ctk.StringVar()
        self.navigation_mode.set("Real")
            
        # Selector de modo condicional
        # Solo se muestra si show_mode_selector es True
        if self.show_mode_selector:
            mode_label = ctk.CTkLabel(
                option_center_frame, 
                text="Que modo desea usar?", 
                font=("Arial", 28, "bold"),
                text_color=self.colors['text_primary'],
                wraplength=300
            )
            mode_label.pack(pady=(0, 20))

            option_menu = ctk.CTkOptionMenu(
                option_center_frame, 
                values=["Real", "Simulacion"], 
                variable=self.navigation_mode, 
                fg_color=self.colors['optionmenu_bg'],
                button_color=self.colors['optionmenu_button'],
                button_hover_color=self.colors['optionmenu_hover'],
                text_color=self.colors['optionmenu_text'],
                font=("Arial", 26),
                width=200,  # Añadido ancho específico
                height=40   # Añadido alto específico
            )
            option_menu.pack()

        # Texto SARA
        # Muestra el nombre del sistema en la parte inferior izquierda
        shop_vision_label = ctk.CTkLabel(
            left_frame, 
            text="SARA", 
            font=('Helvetica', 48, 'bold'),
            text_color=self.colors['text_primary']
        )
        shop_vision_label.pack(side=ctk.BOTTOM, padx=20, pady=20)

        # Frame principal de contenido
        # Contiene los paneles de búsqueda y selección de productos
        main_content = ctk.CTkFrame(right_container, fg_color=self.colors['primary_bg'])
        main_content.pack(fill=ctk.BOTH, expand=True, padx=10, pady=10)

        # Frame de búsqueda
        # Contiene la entrada de texto y botón de búsqueda
        top_search_frame = ctk.CTkFrame(main_content, fg_color=self.colors['primary_bg'], height=40)
        top_search_frame.pack(fill=ctk.X, padx=20, pady=(0, 10))
        top_search_frame.pack_propagate(False)

        search_container = ctk.CTkFrame(top_search_frame, fg_color=self.colors['primary_bg'])
        search_container.pack(expand=True)

        self.search_frame = ctk.CTkFrame(search_container, fg_color=self.colors['primary_bg'])
        self.search_frame.pack()

        # Entry de búsqueda y botón buscar
        # Permite filtrar la lista de productos
        self.search_var = ctk.StringVar()
        self.search_var.trace('w', lambda *args: self.perform_search())

        search_container = ctk.CTkFrame(self.search_frame, fg_color="transparent")
        search_container.pack()

        self.search_entry = ctk.CTkEntry(
            search_container,
            font=("Arial", 20),
            width=550,
            height=40,
            fg_color=self.colors['entry_bg'],
            textvariable=self.search_var
        )
        self.search_entry.pack(side=ctk.LEFT, padx=(0, 10))

        keyboard_button = ctk.CTkButton(
            search_container,
            text="⌨",
            command=lambda: show_virtual_keyboard(self.search_entry, self.root, self.colors),
            width=40,
            height=40,
            fg_color=self.colors['button_bg'],
            text_color=self.colors['button_text'],
            hover_color=self.colors['button_hover']
        )
        keyboard_button.pack(side=ctk.LEFT)

        # Bindear evento Enter para buscar
        self.search_entry.bind('<Return>', lambda e: self.perform_search())
        self.search_frame.bind('<Button-1>', lambda e: 'break')

        # Frame para lista de productos
        # Muestra todos los productos disponibles que coinciden con la búsqueda
        frame1 = ctk.CTkFrame(main_content, fg_color=self.colors['frame_bg'])
        frame1.pack(fill=ctk.BOTH, padx=20, pady=(0, 10), expand=True)

        products_label = ctk.CTkLabel(
            frame1, 
            text="Productos disponibles", 
            font=("Helvetica", 24, "bold"),
            text_color=self.colors['text_primary']
        )
        products_label.pack(pady=10)

        self.treeview_frame = ctk.CTkScrollableFrame(
            frame1, 
            width=300, 
            height=200, 
            fg_color=self.colors['scrollable_frame_bg']
        )
        self.treeview_frame.pack(fill=ctk.BOTH, expand=True, padx=20, pady=10)

        select_button = ctk.CTkButton(
            frame1, 
            text="Seleccionar productos", 
            command=self.select_products, 
            width=200, 
            height=50, 
            font=("Helvetica", 16), 
            fg_color=self.colors['button_bg'],
            text_color=self.colors['button_text'],
            hover_color=self.colors['button_hover']
        )
        select_button.pack(pady=10)

        # Frame para productos seleccionados
        # Muestra los productos que el usuario ha seleccionado
        frame2 = ctk.CTkFrame(main_content, fg_color=self.colors['frame_bg'])
        frame2.pack(fill=ctk.BOTH, padx=20, pady=(10, 20), expand=True)

        selected_label = ctk.CTkLabel(
            frame2, 
            text="Productos Seleccionados", 
            font=("Helvetica", 24, "bold"),
            text_color=self.colors['text_primary']
        )
        selected_label.pack(pady=10)

        self.selected_frame = ctk.CTkScrollableFrame(
            frame2, 
            width=300, 
            height=200, 
            fg_color=self.colors['scrollable_frame_bg']
        )
        self.selected_frame.pack(fill=ctk.BOTH, expand=True, padx=20, pady=10)

        go_to_products_button = ctk.CTkButton(
            frame2, 
            text="Dirigirse a productos", 
            command=self.open_navigation_window, 
            width=200, 
            height=50, 
            font=("Helvetica", 16), 
            fg_color=self.colors['button_bg'],
            text_color=self.colors['button_text'],
            hover_color=self.colors['button_hover']
        )
        go_to_products_button.pack(pady=10)

        self.actualizar_reloj_y_fecha()
        self.refresh_treeview()

    def show_info(self, message, title="Info"):
        # Método para mostrar ventanas de información al usuario
        # Crea una ventana emergente con un mensaje y un botón OK
        info_window = ctk.CTkToplevel()
        info_window.title(title)
        info_window.geometry("300x150")
        
        label = ctk.CTkLabel(
            info_window, 
            text=message, 
            text_color=self.colors['text_primary'],
            padx=20, 
            pady=20
        )
        label.pack(expand=True)
        
        ok_button = ctk.CTkButton(
            info_window, 
            text="OK", 
            command=info_window.destroy, 
            fg_color=self.colors['button_bg'],
            text_color=self.colors['button_text'],
            hover_color=self.colors['button_hover']
        )
        ok_button.pack(pady=10)
    
    def refresh_treeview(self, search_term=""):
        # Método para actualizar la vista de productos
        # Consulta la base de datos y muestra los productos que coinciden
        # con el término de búsqueda
        
        # Limpiar la vista actual
        for widget in self.treeview_frame.winfo_children():
            widget.destroy()

        try:
            db_dir = get_source_db_path('turtlemart', 'products.db')
            self.db_connection = sqlite3.connect(db_dir)
            cursor = self.db_connection.cursor()

            # Añadir logging para debug
            print(f"Searching for: '{search_term}'")
            
            # Modificar la consulta para ser case-insensitive
            cursor.execute("SELECT * FROM products WHERE LOWER(name) LIKE LOWER(?)", 
                        ('%' + search_term + '%',))
            
            results = cursor.fetchall()
            print(f"Found {len(results)} results")

            if not results:
                # Mostrar mensaje cuando no hay resultados
                no_results_label = ctk.CTkLabel(
                    self.treeview_frame,
                    text="No se encontraron productos",
                    font=('Helvetica', 20),
                    text_color=self.colors['text_primary']
                )
                no_results_label.pack(pady=20)
            
            for row in results:
                if row[0] not in self.checkbox_vars:
                    self.checkbox_vars[row[0]] = ctk.IntVar()
                
                checkbox = ctk.CTkCheckBox(
                    self.treeview_frame,
                    font=('Helvetica', 26, 'bold'),
                    text=f"{row[1]}", 
                    variable=self.checkbox_vars[row[0]],
                    fg_color=self.colors['checkbox_bg'],
                    hover_color=self.colors['checkbox_hover']
                )
                checkbox.pack(anchor="w", padx=10, pady=5)

                if self.checkbox_vars[row[0]].get():
                    checkbox.select()
                else:
                    checkbox.deselect()

        except sqlite3.Error as e:
            print(f"Database error: {e}")
            error_label = ctk.CTkLabel(
                self.treeview_frame,
                text=f"Error de base de datos: {str(e)}",
                font=('Helvetica', 20),
                text_color='red'
            )
            error_label.pack(pady=20)
        except Exception as e:
            print(f"Error: {e}")
            error_label = ctk.CTkLabel(
                self.treeview_frame,
                text=f"Error: {str(e)}",
                font=('Helvetica', 20),
                text_color='red'
            )
            error_label.pack(pady=20)
        finally:
            if self.db_connection:
                self.db_connection.close()
                self.db_connection = None

    def select_products(self):
        # Método para guardar los productos seleccionados
        # Guarda en la base de datos los productos marcados por el usuario
        selected_items = [key for key, var in self.checkbox_vars.items() if var.get()]
        if selected_items:
            db_dir = get_source_db_path('turtlemart', 'products.db')
            connection = sqlite3.connect(db_dir)
            cursor = connection.cursor()
            
            cursor.execute("DELETE FROM selected_products")
            
            for product_id in selected_items:
                cursor.execute("SELECT name, x, y FROM products WHERE id = ?", (product_id,))
                product_info = cursor.fetchone()
                cursor.execute("INSERT INTO selected_products (name, x, y) VALUES (?, ?, ?)",
                               (product_info[0], product_info[1], product_info[2]))
            
            connection.commit()
            connection.close()
            self.view_selected_products()
        else:
            self.show_info("No items selected.", title="Error")

    def view_selected_products(self):
        # Método para mostrar los productos seleccionados
        # Recupera y muestra los productos guardados en la base de datos
        for widget in self.selected_frame.winfo_children():
            widget.destroy()

        db_dir = get_source_db_path('turtlemart', 'products.db')
        connection = sqlite3.connect(db_dir)
        cursor = connection.cursor()
        cursor.execute("SELECT * FROM selected_products")
        
        for row in cursor.fetchall():
            ctk.CTkLabel(
                self.selected_frame, 
                text=f"{row[0]}", 
                font=('Helvetica', 26, 'bold'),
                text_color=self.colors['text_primary']
            ).pack(anchor="c", padx=10, pady=5)
        connection.close()

    def perform_search(self):
        # Método para ejecutar la búsqueda de productos
        # Se activa al cambiar el texto en el campo de búsqueda
        search_term = self.search_var.get().strip()
        print(f"Performing search with term: '{search_term}'")
        self.refresh_treeview(search_term)

    def actualizar_reloj_y_fecha(self):
        # Método para actualizar el reloj y la fecha
        # Actualiza los widgets de fecha y hora cada segundo
        now = datetime.datetime.now()
        self.label_reloj.configure(text=now.strftime("%H:%M:%S"))
        self.label_fecha.configure(text=now.strftime("%Y-%m-%d"))
        self.after_id = self.root.after(1000, self.actualizar_reloj_y_fecha)

    def open_navigation_window(self):
        # Método para abrir la ventana de navegación
        # Detiene el reloj y abre la interfaz de navegación
        if self.after_id is not None:
            self.root.after_cancel(self.after_id)
            self.navigation_window = UnifiedNavigationWindow(self, self.navigation_mode.get())
            self.root.withdraw()
            self.navigation_window.mainloop()

    def signal_handler(self, sig, frame):
        # Manejador de señales para interrupciones del sistema
        # Permite detener la aplicación correctamente ante señales SIGINT y SIGTERM
        print("\nSeñal recibida: {sig}")
        self.on_closing()

    def on_closing(self):
        # Método para manejar el cierre de la aplicación
        # Limpia recursos y cierra ordenadamente la aplicación
        if self.is_closing:
            return
            
        try:
            print("\nIniciando proceso de cierre...")
            
            # Limpiar recursos
            self.cleanup_resources()
            
            # Destruir la ventana principal
            if self.root:
                self.root.quit()
                self.root.destroy()
                print("Ventana principal cerrada correctamente")
                
        except Exception as e:
            print(f"Error durante el cierre: {e}")
        finally:
            print("Proceso de cierre completado")
        
    def deiconify(self):
        # Método para restaurar la ventana principal
        # Hace visible la ventana principal después de ocultarla
        self.root.deiconify()
        
    def cleanup_resources(self):
        # Método para liberar todos los recursos antes de cerrar la aplicación
        # Cancela temporizadores, cierra conexiones a bases de datos y destruye ventanas
        """Limpia todos los recursos antes de cerrar"""
        if self.is_closing:
            return
            
        self.is_closing = True
        print("Iniciando limpieza de recursos...")
        
        # Cancelar el temporizador del reloj
        if self.after_id is not None:
            try:
                self.root.after_cancel(self.after_id)
                print("Temporizador del reloj cancelado")
            except Exception as e:
                print(f"Error al cancelar temporizador: {e}")

        # Cerrar conexión a la base de datos si está abierta
        if self.db_connection:
            try:
                self.db_connection.close()
                print("Conexión a la base de datos cerrada")
            except Exception as e:
                print(f"Error al cerrar la base de datos: {e}")

        # Cerrar ventana de navegación si está abierta
        if self.navigation_window:
            try:
                self.navigation_window.destroy()
                print("Ventana de navegación cerrada")
            except Exception as e:
                print(f"Error al cerrar ventana de navegación: {e}")

        print("Limpieza de recursos completada")

if __name__ == '__main__':
    # Punto de entrada principal del programa
    # Inicializa la ventana raíz y crea la instancia del gestor de productos
    root = ctk.CTk()
    app = ProductManager(root)
    root.mainloop()
