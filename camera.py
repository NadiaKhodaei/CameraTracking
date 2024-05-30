import cv2

camera = cv2.VideoCapture(0)

while true:
  ret,frame=camera.read()
  cv2.imshow('frame',frame)
  if cv2.waitKey(1) & 0xFF==ord('q'):
  	break
 camera.release()
 cv2.destroyAllWindows()
