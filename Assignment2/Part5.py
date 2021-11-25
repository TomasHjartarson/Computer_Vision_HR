
import cv2
import numpy as np

 

if __name__ == '__main__' :

    im_src = cv2.imread('PB.PNG')
    im_dst = cv2.imread('PG.PNG')

    #PTS_DST
    #[628, 374]
    #[362, 151]
    #[114, 173]
    #[152, 484]
    
    

    pts_src = np.array([[247,78], [479,185], [423,411], [265,419]])
    
    #pts_src = np.array([[226,185], [479,185], [607,448], [121,473]])
    
    pts_dst = np.array([[166, 121], [397, 117], [618, 371], [109, 415]])
    #pts_dst = np.array([[114, 173], [362, 151], [600, 320], [152, 484]])
    
    
    h, status = cv2.findHomography(pts_src, pts_dst)
    im_out = cv2.warpPerspective(im_src, h, (im_dst.shape[1],im_dst.shape[0]))

    cv2.imshow("Source Image", im_src)
    cv2.imshow("Destination Image", im_dst)
    cv2.imshow("Warped Source Image", im_out)

    cv2.waitKey(0)




