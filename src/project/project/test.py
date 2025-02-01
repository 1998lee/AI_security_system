import cv2
import os
from ultralytics import YOLO

hasattr
# 모델 로드
model = YOLO('/home/inyoung/Downloads/best.pt')

# 카메라 연결
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("카메라 연결 실패")
    exit()
# 해상도 설정
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
# 설정된 해상도 확인
width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
print(f"설정된 해상도: {width}x{height}")
# 색상 정의
class_colors = {
    "car": (0, 255, 0),
    "dummy" : (255, 0 , 0)
}
# class_colors = {
#     "red": (0, 0, 255),
#     "blue": (255, 0, 0),
#     "purple": (128, 0, 128),
# }
while True:
    ret, frame = cap.read()
    if not ret:
        print("프레임을 읽을 수 없습니다.")
        break
    # 객체 인식
    results = model.predict(source=frame, conf=0.5, device='cpu')  # confidence threshold 0.5
    for result in results:
        boxes = result.boxes  # 감지된 객체의 경계 상자
        for box in boxes:
            # 경계 상자와 클래스 정보 추출
            x1, y1, x2, y2 = map(int, box.xyxy[0])  # 좌표
            conf = box.conf[0]  # confidence score
            cls = int(box.cls[0])  # class index
            label = model.names[cls]  # 클래스 이름
            # 색상 설정
            color = class_colors.get(label, (0, 255, 0))  # 기본 색상 초록색
            # 경계 상자 그리기
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            # 텍스트 추가
            text = f"{label} {conf:.2f}"
            cv2.putText(frame, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    # 화면에 표시
    cv2.imshow('Camera', frame)
    key = cv2.waitKey(1) & 0xFF
    # 'q' 키를 눌러 종료
    if key == ord('q'):
        break
    
# 리소스 해제
cap.release()
cv2.destroyAllWindows()