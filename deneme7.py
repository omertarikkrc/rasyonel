import cv2
import numpy as np

cam = cv2.VideoCapture(0)
lower_red = (0,137,135)
upper_red = (180,255,237)

def getContours(binary_image):

    contours, hierarchy = cv2.findContours(binary_image.copy(),
                                              cv2.RETR_EXTERNAL,
                                              cv2.CHAIN_APPROX_SIMPLE)
    return contours

def area_contour(binary_image,final_frame, contours):
    black_image = np.zeros([binary_image.shape[0], binary_image.shape[1],3],'uint8')

    for c in contours:
        area = cv2.contourArea(c)
        perimeter = cv2.arcLength(c,True)
        ((x,y), radius) = cv2.minEnclosingCircle(c)
        if (area>500):
            cv2.drawContours(final_frame, [c], -1, (150,250,150), 1)
            cv2.drawContours(black_image, [c], -1, (150,250,150), 1)
            cx, cy = get_contour_center(c)
            cv2.circle(final_frame, (cx,cy),(int)(radius),(0,0,255),1)
            cv2.circle(black_image, (cx,cy),(int)(radius),(0,0,255),1)
            cv2.circle(black_image, (cx,cy),5 ,(150,150,255),-1)
            print("x: {}, y: {}".format(cx,cy))
            #print("Area: {}, Perimeter: {}".format(area,perimeter))
    #print("Number of Contours: {}".format(len(contours)))
    cv2.imshow("Cam Contours",final_frame )
    cv2.imshow("Black Image Contours", black_image)

def get_contour_center(contour):
    M = cv2.moments(contour)
    cx = -1
    cy = -1
    if (M['m00'] != 0):
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
    return cx, cy 

def main():

    cam = cv2.VideoCapture(0)
    lower_red = (0,137,135)
    upper_red = (180,255,237)

    while(cam.isOpened()):
        ret, frame = cam.read()
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_red,upper_red)
        final_frame = cv2.bitwise_and(frame,frame, mask = mask)

        cv2.imshow("Frame", frame)
        cv2.imshow("Mask", mask)
        cv2.imshow("Final", final_frame)
        
        area_contour(mask, final_frame, getContours(mask))

        if cv2.waitKey(10)  & 0xFF == ord('q'):
            break

    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

cam.release()
cv2.destroyAllWindows()








