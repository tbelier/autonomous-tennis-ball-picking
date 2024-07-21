import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


class DetectRobot(Node):

    def __init__(self):
        super().__init__('detect_robot')

        self.threshold = 0.5
        self.cv_bridge = CvBridge()

        self.subscription = self.create_subscription(Image, '/zenith_camera/image_raw', self.camera_callback, 10)
        self.image_publisher = self.create_publisher(Image, '/image_detection_robot', 10)
        self.XY_publisher = self.create_publisher(PoseStamped, '/xy_robot',10)

    # on détecte le robot
    def detect_rose(self,imgBGR):
        # Conversion BGR vers HSV
        imgHSV = cv2.cvtColor(imgBGR, cv2.COLOR_RGB2HSV)
        # Seuils pour detection du rose
        Hmin = 120
        Hmax = 170
        Smin = 60
        Smax = 255
        Vmin = 0
        Vmax = 255
        # Seuillage HSV pour detection du rose
        imgbin = cv2.inRange(imgHSV, (Hmin, Smin, Vmin), (Hmax, Smax, Vmax))

        return imgbin



    # on récupère les coordonnées du robot
    def camera_callback(self, msg):
        # on initialise le message à publier
        msgtopublish = PoseStamped()
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg)
        imgbin = self.detect_rose(cv_image)

        image_msg = self.cv_bridge.cv2_to_imgmsg(imgbin)
        self.image_publisher.publish(image_msg)

        # on récupère les contours du robot
        contours, hierarchy = cv2.findContours(imgbin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #print("contours :" + str(contours))

        # on dessine les contours du robot

        if len(contours) > 0: # si on détecte un contour
            # on parcourt la liste des contours
            for contour in contours:
                cv2.drawContours(imgbin, [contour], -1, (0,255,0), 5)

                M = cv2.moments(contour)
                #print("moment : " + str(M["m00"]))

                # on calcule les coordonnées du barycentre
                if M["m00"] != 0:
                    cx = float(M["m10"] / M["m00"])
                    cy = float(M["m01"] / M["m00"])
                    msgtopublish.pose.position.x = cx
                    msgtopublish.pose.position.y = cy
                    self.XY_publisher.publish(msgtopublish)
                # si le moment est trop faible, on affiche un message
                else:
                    print("Le moment est trop faible")
        # si on ne détecte pas de contour, on affiche un message
        else:
            print("je ne detecte pas de contour")
        print("--- Robot ---")
        print("Lx :" + str(cx))
        print("Ly :" + str(cy))

    # on détecte le robot avec la caméra de visualisation
    def camera_callback_visu(self, msg):

        cv_image = self.cv_bridge.imgmsg_to_cv2(msg)
        imgbin = self.detect_rose(cv_image)
        # on récupère les contours du robot
        contours, hierarchy = cv2.findContours(imgbin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        img_contours = cv_image.copy()
        color = (0, 255, 0)  # Vert
        thickness = 2

        cv2.drawContours(img_contours, contours, -1, color, thickness)
        print("contour : "+ str(contours[0]))
        # recherche du plus grand contour
        best_contour = contours[0]
        max_area = cv2.contourArea(best_contour)
        # on parcourt la liste des contours pour trouver le plus grand
        for contour in contours:
            area = cv2.contourArea(contour)

            if area > max_area:
                best_contour = contour
                max_area = area

        img_contour = cv_image.copy()
        cv2.drawContours(img_contour, [best_contour], -1, color, thickness)

        if len(contours) > 0:
            # Selectionnez le premier contour (l'objet) dans la liste des contours
            M = cv2.moments(best_contour)
            print("M :" + str(M))
            # Calculez les coordonnees du barycentre
            if M["m00"] != 0:

                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                print("Coordonnees du barycentre : ({}, {})".format(cx, cy))

                center = (int(cx), int(cy))
                cv2.circle(cv_image, center, 2, (0, 255, 255), -1)  # Dessine un cercle rouge sur le barycentre
                #image_msg = self.cv_bridge.cv2_to_imgmsg(cv_image)
                #self.image_publisher.publish(image_msg)
            else:
                print("L'objet a une aire nulle, impossible de calculer le barycentre.")
        else:
            print("Aucun contour trouve dans l'image.")





def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = DetectRobot()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

