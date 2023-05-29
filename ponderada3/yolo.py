from ultralytics import YOLO 
import cv2

model = YOLO("ponderada3/best.pt")

def get_yolo_results(img):
    imagem = cv2.imread(img)
    results = model(imagem)
    annotated_frame = results[0].plot()
    cv2.imshow("YOLOv8 Inference", annotated_frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return results