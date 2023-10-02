import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import matplotlib.pyplot as plt


class LidarReader(Node):
    def __init__(self):

        '''
            Recebe como parâmetro o tópico /scan (tipo LaserScan) em que nele contem:

            ranges: É um array que contém as distâncias medidas pelo LIDAR em diferentes ângulos.
            Cada elemento do array corresponde à distância medida em um ângulo específico.

            angle_min: É o ângulo mínimo que o LIDAR pode cobrir. 
            O primeiro elemento do array ranges corresponde a esse ângulo.

            angle_max: É o ângulo máximo (em radianos) que o LIDAR pode cobrir. 
            O último elemento do array ranges corresponde a esse ângulo.

            angle_increment: É o incremento angular entre as leituras 
            consecutivas no array ranges. Isso significa que você pode calcular o ângulo
            correspondente de cada elemento no array ranges usando o índice do elemento e 
            o valor de angle_increment.

            range_min: É a distância mínima  que o LIDAR pode medir. 

            range_max: É a distância máxima  que o LIDAR pode medir.
        
        
        '''
        super().__init__('lidar_reader')
        self.get_logger().info('lidar_reader node initialized')


        self.sub_back_left = self.create_subscription(
            LaserScan, '/scan', self.plot, 10)

    def plot(self, msg):
        X_axis = []
        Y_axis = []

        plt.close()
        fig, ax = plt.subplots()
        ax.clear()

        ranges = msg.ranges
        angle_increment = msg.angle_increment
        for angle in range(int(len(ranges))):
            if not math.isinf(ranges[angle]):
                X_axis.append(ranges[angle] * np.cos(-np.pi + (angle * angle_increment)))
                Y_axis.append(ranges[angle] * np.sin(-np.pi + (angle * angle_increment)))

        
        plt.scatter(X_axis, Y_axis, color='blue')
        
        plt.xlim(-5, 5)
        plt.ylim(-5, 5)
        plt.savefig('Lidar.png')
        plt.draw()



def main(args=None):
    rclpy.init(args=args)
    node = LidarReader()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()