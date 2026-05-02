#node de suscrtipcio de un IMU per a trobar la posicio de un cotxe mitjançant Float 32MultiArray, el node es subscriu a un topic on el IMU publica les dades 
#ax acceleracio en x
#ay acceleracio en y
#az acceleracio en z
#vgx velocitat angular en x
#vgy velocitat angular en y
#vgz velocitat angular en z
#gzx angle entre els eixos x i z
#gzy angle entre els eixos y i z
#gxy angle entre els eixos x i y
#v velocitat del cotxe
#x i y posicio del cotxe en el pla, es calcula a partir de les dades de l'IMU i es publica en un topic per a que altres nodes puguin utilitzar aquesta informacio per a la navegacio del cotxe.

import rclpy
import math
from rclpy.node import Node
from my_pakage.msg import IMU_reader, IMU_transformed


class IMUSuscriber(Node):
    def __init__(self):
        super().__init__('imu_suscriber')
        self.ap = 0.0
        self.gzx = 0.0
        self.gzy = 0.0
        self.gxy = 0.0
        self.v = 0.0
        self.x = 0.0
        self.y = 0.0
        
        self.imu_subscriber = self.create_subscription(
            IMU_reader,
            'imu_readers',
            self.listener_callback,
            10
        )
        self.imu_publisher = self.create_publisher(IMU_transformed, 'imu_transformed', 10)

    def start_position(self, msg):
        """Calcula els angles inicials a partir de les dades de l'IMU"""
        try:
            self.gzx = (math.acos(msg.ax / msg.az)) * 180 / math.pi
        except:
            self.gzx = 0.0
        try:
            self.gzy = (math.acos(msg.ay / msg.az)) * 180 / math.pi
        except:
            self.gzy = 0.0
        try:
            self.gxy = (math.acos(msg.ax / msg.ay)) * 180 / math.pi
        except:
            self.gxy = 0.0
        
        self.ap = (msg.ax**2 + msg.ay**2 + msg.az**2)**0.5

    def listener_callback(self, msg):
        axp = 0.0
        ayp = 0.0
        azp = 0.0
        """Processa dades de l'IMU i calcula posicio"""
        if self.gzx == 0.0 and self.gzy == 0.0:
            self.start_position(msg)
        
        # Actualitza els angles segons velocitats angulars
        if self.gxy < 90:

            self.gzx = (msg.vgx*0.1 + self.gzx + msg.vgz*0.1*(self.gzx+self.gzy)/360) % 360
            self.gzy = (msg.vgy*0.1 + self.gzy + msg.vgz*0.1*(self.gzy+self.gzx)/360) % 360       
            self.gxy = (msg.vgz*0.1 + self.gxy) % 360
        elif self.gxy > 90 and self.gxy < 270:
            x = 0
        #ERROR: no es pot calcular angles quan gxy esta entre 90 i 270, ja que el cosinus de l'angle seria negatiu i no es podria calcular l'angle a partir de les dades de l'IMU, en aquest cas es podria utilitzar un altre metode per a calcular els angles, com per exemple utilitzar les velocitats angulars per a actualitzar els angles en lloc de calcular-los a partir de les dades de l'IMU, o utilitzar un filtre de Kalman per a fusionar les dades de l'IMU amb altres sensors per a obtenir una estimacio mes precisa dels angles i la posicio del cotxe.
        #TODO: actualitzar angles segons velocitats angulars en aquest cas
        
        
        # Calcula acceleracio per gravetat
        axp = math.cos(self.gzx*2*math.pi/360)*self.ap
        ayp = math.cos(self.gzy*2*math.pi/360)*self.ap
        azp = math.cos(self.gxy*2*math.pi/360)*self.ap
        
        # Calcula velocitat i posicio
        self.v = (msg.ax - axp)*0.1 + self.v
        self.x = self.v*math.cos(self.gzx*2*math.pi/360)*0.1 + self.x
        self.y = self.v*math.sin(self.gzx*2*math.pi/360)*0.1 + self.y
        
        # Publica resultat
        output_msg = IMU_transformed()
        output_msg.ax = msg.ax
        output_msg.ay = msg.ay
        output_msg.az = msg.az
        output_msg.v = self.v
        output_msg.x = self.x
        output_msg.y = self.y
        output_msg.gzx = self.gzx
        output_msg.gzy = self.gzy
        output_msg.gxy = self.gxy
        
        self.imu_publisher.publish(output_msg)


def main(args=None):
    try:
        rclpy.init(args=args)
        imu_suscriber = IMUSuscriber()
        rclpy.spin(imu_suscriber)
    except KeyboardInterrupt:
        print("exit node")
    except Exception as e:
        print(e)

        
if __name__ == '__main__':
    main()
