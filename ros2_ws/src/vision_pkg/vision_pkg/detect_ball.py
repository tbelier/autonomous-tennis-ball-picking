import rclpy
from rclpy.node import Node
from custom_msg.msg import ImgDetection
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

# ce noeud permet de détecter la balle
class DetectBall(Node):

    def __init__(self):
        super().__init__('detect_ball')
        # on s'abonne au topic /zenith_camera/image_raw
        self.threshold = 0.5
        self.cv_bridge = CvBridge()

        self.subscription = self.create_subscription(Image, '/zenith_camera/image_raw', self.camera_callback, 10)
        self.image_publisher = self.create_publisher(Image, '/image_detection_ball', 10)
        self.XY_publisher = self.create_publisher(ImgDetection, '/xy_ball',10)

    # on détecte la balle
    def detect_yellow(self,imgBGR):
        # Conversion BGR vers HSV
        imgHSV = cv2.cvtColor(imgBGR, cv2.COLOR_RGB2HSV)
        # Seuils pour detection du jaune
        Hmin = 25
        Hmax = 35
        Smin = 60
        Smax = 255
        Vmin = 0
        Vmax = 255

        # Seuillage HSV pour detection du jaune

        imgbin = cv2.inRange(imgHSV, (Hmin, Smin, Vmin), (Hmax, Smax, Vmax))

        return imgbin



    # on récupère les coordonnées de la balle
    def camera_callback(self, msg):
        # on initialise le message à publier
        msgtopublish = ImgDetection()
        # on initialise les coordonnées de la balle
        Lx = []
        Ly = []
        # on récupère l'image
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg)
        # on détecte la balle
        imgbin = self.detect_yellow(cv_image)
        # on publie l'image
        image_msg = self.cv_bridge.cv2_to_imgmsg(imgbin)
        self.image_publisher.publish(image_msg)

        # on récupère les contours de la balle
        contours, hierarchy = cv2.findContours(imgbin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #print("contours :" + str(contours))
        # on dessine les contours de la balle
        if len(contours) > 0:
            # Selectionnez le premier contour (l'objet) dans la liste des contours
            for contour in contours:

                cv2.drawContours(imgbin, [contour], -1, (0,255,0), 5)
                # Calculez les coordonnees du barycentre
                M = cv2.moments(contour)
                #print("moment : " + str(M["m00"]))
                # condition pour éviter les erreurs
                if M["m00"] != 0:
                    # on récupère les coordonnées du barycentre
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    # on publie les coordonnées de la balle
                    Lx.append(cx)
                    Ly.append(cy)

                    msgtopublish.coordx = Lx
                    msgtopublish.coordy = Ly
                    #cv2.rectangle(imgbin, (cx - 5, cy - 5), (cx + 5, cy + 5), (255, 0, 0), 2)
                    self.XY_publisher.publish(msgtopublish)
                # si le moment est trop faible, on affiche un message
                else:
                    print("Le moment ball est trop faible")
        # si la balle n'est pas détectée, on affiche un message
        else:
            print("je ne detecte pas de contour")
        print("--- Ball ---")
        print("Lx :" + str(Lx))
        print("Ly :" + str(Ly))


    # on récupère les coordonnées de la balle
    def camera_callback_visu(self, msg):

        cv_image = self.cv_bridge.imgmsg_to_cv2(msg)
        imgbin = self.detect_yellow(cv_image)
        contours, hierarchy = cv2.findContours(imgbin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        img_contours = cv_image.copy()
        color = (0, 255, 0)  # Vert
        thickness = 2

        cv2.drawContours(img_contours, contours, -1, color, thickness)
        print("contour : "+ str(contours[0]))
        # Selectionnez le premier contour (l'objet) dans la liste des contours
        best_contour = contours[0]
        # Calculez l'aire du contour
        max_area = cv2.contourArea(best_contour)
        # Parcourez la liste des contours
        for contour in contours:
            area = cv2.contourArea(contour)
            # Si l'aire du contour courant est plus grande que l'aire du meilleur contour
            if area > max_area:
                best_contour = contour
                max_area = area
        # Dessinez le meilleur contour
        img_contour = cv_image.copy()
        cv2.drawContours(img_contour, [best_contour], -1, color, thickness)
        # si la balle est détectée, on affiche les coordonnées de la balle
        if len(contours) > 0:
            # Selectionnez le premier contour (l'objet) dans la liste des contours
            M = cv2.moments(best_contour)
            print("M :" + str(M))
            # Calculez les coordonnees du barycentre
            if M["m00"] != 0:

                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                print("Coordonnees du barycentre : ({}, {})".format(cx, cy))
                # Dessinez un cercle sur le barycentre
                center = (int(cx), int(cy))
                cv2.circle(cv_image, center, 2, (0, 255, 255), -1)  # Dessine un cercle rouge sur le barycentre
                #image_msg = self.cv_bridge.cv2_to_imgmsg(cv_image)
                #self.image_publisher.publish(image_msg)
            else:
                print("L'objet a une aire nulle, impossible de calculer le barycentre.")
        # si la balle n'est pas détectée, on affiche un message
        else:
            print("Aucun contour trouve dans l'image.")





def main(args=None):
    print("Node before start")
    rclpy.init(args=args)
    print("Node after start")
    minimal_publisher = DetectBall()
    print("init Detect Ball")
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

