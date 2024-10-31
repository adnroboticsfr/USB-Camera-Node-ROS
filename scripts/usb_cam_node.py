#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import time

def main():
    # Initialiser le nœud ROS
    rospy.init_node('usb_cam_node', anonymous=True)

    # Récupérer les paramètres
    camera_index = rospy.get_param("~camera_index", 0)
    rate_hz = rospy.get_param("~rate_hz", 10)
    flip_image = rospy.get_param("~flip_image", True)
    width = rospy.get_param("~width", 640)
    height = rospy.get_param("~height", 480)

    # Créer un éditeur pour publier des images et l'état de connexion
    image_pub = rospy.Publisher('usb_cam/image_raw', Image, queue_size=10)
    connection_status_pub = rospy.Publisher('usb_cam/connected', Bool, queue_size=1)

    # Créer un bridge OpenCV <-> ROS
    bridge = CvBridge()

    # Ouvrir la caméra USB
    cap = cv2.VideoCapture(camera_index)
    retry_count = 0
    max_retries = 5

    # Essayer plusieurs fois d'ouvrir la caméra si elle échoue
    while not cap.isOpened() and retry_count < max_retries:
        rospy.logwarn("Attempting to open camera...")
        rospy.sleep(1)
        cap.open(camera_index)
        retry_count += 1

    if not cap.isOpened():
        rospy.logerr("Unable to open the camera after multiple attempts.")
        return

    # Régler la résolution de la caméra
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    rate = rospy.Rate(rate_hz)  # Fréquence définie par le paramètre rate_hz
    connection_status = True

    # Boucle principale
    while not rospy.is_shutdown():
        start_time = time.time()
        
        # Capturer une image
        ret, frame = cap.read()
        
        # Si la capture échoue, publier l'état de déconnexion et continuer
        if not ret:
            rospy.logwarn("Failed to capture image.")
            connection_status = False
            connection_status_pub.publish(connection_status)
            continue

        # Publier l'état de connexion de la caméra
        connection_status = True
        connection_status_pub.publish(connection_status)

        # Retourner l'image si demandé
        if flip_image:
            frame = cv2.flip(frame, 0)

        try:
            # Convertir l'image de OpenCV à ROS et la publier
            image_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            image_pub.publish(image_msg)
        except Exception as e:
            rospy.logerr(f"Error converting the image: {e}")
            connection_status = False
            connection_status_pub.publish(connection_status)

        # Calculer et afficher le temps de traitement de l'image
        end_time = time.time()
        processing_time = end_time - start_time
        rospy.loginfo(f"Processing time per frame: {processing_time:.4f} seconds")

        rate.sleep()

    # Libérer la caméra
    cap.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
