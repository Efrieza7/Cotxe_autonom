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
#axp acceleracio en x del cotxe produida pel pes del cotxe
#ayp acceleracio en y del cotxe produida pel pes del cotxe
#azp acceleracio en z del cotxe produida pel pes del cotxe

#x i y posicio del cotxe en el pla, es calcula a partir de les dades de l'IMU i es publica en un topic per a que altres nodes puguin utilitzar aquesta informacio per a la navegacio del cotxe.
import rclpy
import math
from rclpy.node import Node
from my_pakage.msg import IMU


class IMUSuscriber(Node):
    def __init__(self):
        super().__init__('imu_suscriber')
        self.start_position
        #falta el publixher i el test
        self.imu_subscriver = self.create_subscription(
            IMU,
            'IMU_data',
            self.listener_callback,
            10
        )
        self.imu_publisher = self.create_publisher(IMU,'IMU_data',10)

    def start_position(self, msg):

        msg.gzx = (math.arccosine(msg.ax/msg.az))
        msg.gzy = (math.arccosine(msg.ay/msg.az))
        msg.gxy = (math.arccosine(msg.ax/msg.ay))

        msg.axp = msg.ax
        msg.ayp = msg.ay
        msg.azp = msg.az

        return msg.gzx, msg.gzy, msg.gxy, msg.axp, msg.ayp, msg.azp
    
    def listener_callback(self, msg):

        msg.gzx = msg.vgx*0.1 + msg.gzx
        msg.gzy = msg.vgy*0.1 + msg.gzy
        msg.gxy = msg.vgz*0.1 + msg.gxy
        
        msg.axp = math.cos(msg.gzx)*msg.axp + math.cos(msg.gzy)*msg.ayp + math.cos(msg.gxy)*msg.azp
        msg.ayp = math.cos(msg.gzx)*msg.axp + math.cos(msg.gzy)*msg.ayp + math.cos(msg.gxy)*msg.azp
        msg.azp = math.cos(msg.gzx)*msg.axp + math.cos(msg.gzy)*msg.ayp + math.cos(msg.gxy)*msg.azp

        msg.v = (msg.ax-msg.axp)*0.1 + msg.v
        msg.x = msg.v*math.cos(msg.gyx)
        msg.y = msg.v*math.sin(msg.gyx)


        


        


        self.imu_publisher.publish(msg)
                

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
