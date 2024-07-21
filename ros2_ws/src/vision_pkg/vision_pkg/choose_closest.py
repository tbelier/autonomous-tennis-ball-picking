import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from custom_msg.msg import ImgDetection
import numpy as np

# ce noeud permet de choisir la balle la plus proche du robot
class ChooseClosest(Node):
    def __init__(self):
        super().__init__('choose_closest')
        # on s'abonne au topic /xy_ball
        self.ball_subscription = self.create_subscription(ImgDetection, '/xy_ball', self.ball_callback, 10)
        # on s'abonne au topic /xy_robot
        self.robot_subscription = self.create_subscription(PoseStamped, '/xy_robot', self.robot_callback, 10)
        # on publie sur le topic /xy_closest_ball
        self.closest_publisher = self.create_publisher(PoseStamped, '/xy_closest_ball',10)
        # on initialise les coordonnées du robot et de la balle
        self.robot_coordx = 0
        self.robot_coordy = 0
        self.ball_coordx = []
        self.ball_coordy = []
        self.ball_coordx_bis = []
        self.ball_coordy_bis = []
        
        self.x_closest = 0
        self.y_closest = 0
        self.img_w = 1280   # taille de l'image
        
    # on récupère les coordonnées de la balle et du robot
    def ball_callback(self, msg):
        # on vérifie que la balle est détectée
        if len(msg.coordx) > 0:
            self.ball_coordx = msg.coordx
            self.ball_coordy = msg.coordy
            ball_on_same_side = ((self.img_w/2 - np.array(self.ball_coordx)) * (self.img_w/2 - self.robot_coordx)) > 0
            # self.get_logger().info("ball_on_same_side = "+str(ball_on_same_side))
            if ball_on_same_side.any():
                itemindex = np.where(ball_on_same_side == True)
                itemindex = itemindex[0]
                # self.get_logger().info("itemindex = "+str(itemindex))
                
                self.ball_coordx_bis = np.array(self.ball_coordx)[itemindex]
                self.ball_coordy_bis = np.array(self.ball_coordy)[itemindex]
            else:
                self.ball_coordx_bis = self.ball_coordx
                self.ball_coordy_bis = self.ball_coordy
            self.chooseClosest()
        # si la balle n'est pas détectée, on affiche un message
        else :
            print("En attente de detection de balles ...")

        print("--- Ball ---")
        print("Lx :" + str(self.ball_coordx_bis))
        print("Ly :" + str(self.ball_coordy_bis))
        
    # on récupère les coordonnées du robot
    def robot_callback(self, msg):
        # on vérifie que le robot est détecté
        if msg.pose.position.x != 0:
            self.robot_coordx = msg.pose.position.x
            self.robot_coordy = msg.pose.position.y
        # si le robot n'est pas détecté, on affiche un message
        else:
            print("En attente de detection du robot ...")

        print("--- Robot ---")
        print("Lx :" + str(self.robot_coordx))
        print("Ly :" + str(self.robot_coordy))
        
    # on choisit la balle la plus proche du robot
    def chooseClosest(self):
        # on initialise le message à publier
        msgtopublish = PoseStamped()
        # on vérifie que le robot et la balle sont détectés
        if (self.robot_coordy != 0 and self.robot_coordx != 0):
            d_closest = np.sqrt((self.robot_coordx-self.ball_coordx_bis[0])**2+(self.robot_coordy-self.ball_coordy_bis[0])**2)
            # on parcourt la liste des balles détectées
            for k in range(len(self.ball_coordx_bis)):
                # on calcule la distance entre le robot et la balle
                if np.sqrt((self.robot_coordx-self.ball_coordx_bis[k])**2+(self.robot_coordy-self.ball_coordy_bis[k])**2) <= d_closest:
                    self.x_closest = self.ball_coordx_bis[k]
                    self.y_closest = self.ball_coordy_bis[k]
            # on publie les coordonnées de la balle la plus proche
            msgtopublish.pose.position.x = float(self.x_closest)
            msgtopublish.pose.position.y = float(self.y_closest)
            self.closest_publisher.publish(msgtopublish)

            print("--- La balle la plus proche se trouve aux coordonées : ")
            print("x = " + str(self.x_closest))
            print("y = " + str(self.y_closest))
        # si le robot ou la balle ne sont pas détectés, on affiche un message
        else :
            print("wait")

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = ChooseClosest()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
