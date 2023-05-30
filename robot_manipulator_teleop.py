#! /usr/bin/env python

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String

from pynput import keyboard as kb

import tkinter as tk
from tkinter import filedialog

from time import perf_counter
import serial,time

class teleop(Node):
    """Crea objetos de tipo nodo."""
    
    def __init__(self):
        """Constructor de la clase teleop."""

        super().__init__("robot_manipulator_teleop")
        self.angulo = 5

        self.cmd_manipulator_key = self.create_publisher(String,'/manipulator_key',10)
        self.get_logger().info("Manipulator Teleop has been started correctly.")

    def print_instructions(self):
        """Imprime en la terminal las instrucciones de uso."""

        print("________________________________________________________________")
        print("                        Instrucciones")
        print("    Para mover la juntura deseada presione las teclas según se indica.")
        print("--------------------------JUNTURA 1-------------------------")
        print("         Presione 'T' para ir hacia adelante.")
        print("         Presione 'F' para ir hacia atras.")
        print("--------------------------JUNTURA 2-------------------------")
        print("         Presione 'Y' para ir hacia adelante.")
        print("         Presione 'G' para ir hacia atras.")
        print("--------------------------JUNTURA 3-------------------------")
        print("         Presione 'U' para ir hacia adelante.")
        print("         Presione 'H' para ir hacia atras.")
        print("--------------------------JUNTURA 4-------------------------")
        print("         Presione 'I' para ir hacia adelante.")
        print("         Presione 'J' para ir hacia atras.")
        print("________________________________________________________________")
        
    def receive_parameters(self):
        """Pide los parámetros de velocidad lineal y angular al usuario y los publica en el tópico '/turtlebot_route'"""
        
        self.vel_1 = float(input("[INFO] Indique la velocidad deseada para la juntura 1: "))
        self.vel_2 = float(input("[INFO] Indique la velocidad deseada para la juntura 2: "))
        self.vel_3 = float(input("[INFO] Indique la velocidad deseada para la juntura 3: "))
        self.vel_4 = float(input("[INFO] Indique la velocidad deseada para la juntura 4: "))

        self.print_instructions()
        
    def key_callback(self,angulo,num):
        """Multiplica la velocidad lineal y angular por -1 o 1 dependiendo de la tecla presionada. Publica el
        mensaje tipo Twist en el tópico '/turtlebot_cmdVel'."""

        string_mss = String()
        string_mss.data = f"{angulo};{num}"
        self.cmd_manipulator_key.publish(string_mss)
        
    def stops_movement(self):
        """Detiene el movimiento del robot cuando se deja de presionar una tecla. Publica el mensaje tipo
        Twist en el tópico '/turtlebot_cmdVel' con velocidad lineal y angular en cero."""

        a=2

    def on_press(self,key):
        """Cuando se presiona una tecla en el teclado, si es 'w','a','s' o 'd' asigna un valor a las variables
        a y l que multiplican por 1 o -1 las velocidades lineales y angulares respectivamente.
        
        Args:
            key: tecla presionada en el teclado
        """
        try:
            pos_orientation = ['t','y','u','i']
            neg_orientation = ['f','g','h','j']

            orientation = 0
            num = 0
            angulo = 0
            
            if key.char in pos_orientation:
                orientation=1
                if key.char == 't':
                    num = 1
                    angulo = self.vel_1
                elif key.char =='y':
                    num = 2
                    angulo = self.vel_2
                elif key.char == 'u':
                    num = 3
                    angulo = self.vel_3
                else:
                    num = 4
                    angulo = self.vel_4
                self.key_callback(angulo,num)

            elif key.char in neg_orientation:
                if key.char == 'f':
                    num = 1
                    angulo = self.vel_1
                elif key.char =='g':
                    num = 2
                    angulo = self.vel_2
                elif key.char == 'h':
                    num = 3
                    angulo = self.vel_3
                else:
                    num = 4
                    angulo = self.vel_4
                self.key_callback(-1*angulo,num)
            else:
                print("[INFO] La tecla presionada no tiene un movimiento asociado \n Siga las instrucciones.")
            
        except:
            print("Caracter especial no identificado.")
        
    def on_release(self,key):
        """Cuando se deja de presionar la tecla, llama a la función que detiene el movimiento del robot.
        Publica en el tópico '/turtlebot_route' el String que contiene la tecla presionada y el tiempo que
        se presionó.
        
        Args:
            key: tecla que se dejo de presionar en el teclado.
        """
        a=2
        
    def listen_keyboard(self):
        """Ecucha el teclado y esta a la espera de que se presione una tecla para ejecutar alguna función."""
        with kb.Listener(on_press=self.on_press,on_release=self.on_release) as listener:
            listener.join()

def main(args=None):
    rclpy.init(args=args)
    teleop_node = teleop()
    
    teleop_node.receive_parameters()
    teleop_node.listen_keyboard()
    
    rclpy.spin(teleop_node)
    
    teleop.destroy_node()
    rclpy.shutdown()
    
if __name__== "__main__":
    main()