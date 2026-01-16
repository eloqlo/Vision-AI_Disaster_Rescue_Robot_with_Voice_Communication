import cv2
import numpy as np
from ultralytics import YOLO
from collections import deque
import threading
import time
from queue import Queue
from dataclasses import dataclass
from typing import Optional, Tuple
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@dataclass
class DetectionResult:
    """Detection 결과를 담는 데이터클래스"""
    frame: np.ndarray
    boxes: np.ndarray
    confidences: np.ndarray
    class_ids: np.ndarray
    timestamp: float
    inference_time: float


class CameraThread(threading.Thread):
    """Pi Camera v2에서 프레임을 읽는 스레드"""
    
    def __init__(self, queue: Queue, resolution: Tuple[int, int] = (640, 480)):
        super().__init__(daemon=True)
        self.queue = queue
        self.resolution = resolution
        self.cap = None
        self.running = False
        
    def run(self):
        """카메라에서 프레임 캡처"""
        try:
            # picamera2 사용 (Pi OS bullseye 이상)
            from picamera2 import Picamera2
            
            self.cap = Picamera2()
            config = self.cap.create_preview_configuration(
                main={"format": 'RGB888', "size": self.resolution}
            )
            self.cap.configure(config)
            self.cap.start()
            self.running = True
            
            logger.info(f"Camera started: {self.resolution}")
            
            while self.running:
                frame = self.cap.capture_array()
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                
                # 큐가 너무 차면 가장 오래된 프레임 제거 (지연 방지)
                if self.queue.qsize() > 2:
                    try:
                        self.queue.get_nowait()
                    except:
                        pass
                
                self.queue.put((frame, time.time()))
                
        except ImportError:
            logger.warning("picamera2 not available, falling back to OpenCV")
            self.cap = cv2.VideoCapture(0)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
            self.running = True
            
            while self.running:
                ret, frame = self.cap.read()
                if ret:
                    if self.queue.qsize() > 2:
                        try:
                            self.queue.get_nowait()
                        except:
                            pass
                    self.queue.put((frame, time.time()))
                    
        except Exception as e:
            logger.error(f"Camera error: {e}")
        finally:
            if self.cap:
                self.cap.release()
            self.running = False
    
    def stop(self):
        self.running = False


class InferenceThread(threading.Thread):
    """YOLOv8 inference를 수행하는 스레드"""
    
    def __init__(self, input_queue: Queue, output_queue: Queue, model_path: str = "yolov8n.pt"):
        super().__init__(daemon=True)
        self.input_queue = input_queue
        self.output_queue = output_queue
        self.model = YOLO(model_path)
        self.running = False
        
    def run(self):
        """YOLOv8 inference 수행"""
        self.running = True
        logger.info("Inference thread started")
        
        try:
            while self.running:
                try:
                    frame, timestamp = self.input_queue.get(timeout=1)
                    
                    start_time = time.time()
                    results = self.model(frame, conf=0.5, verbose=False)
                    inference_time = time.time() - start_time
                    
                    # 결과 파싱
                    result = results[0]
                    boxes = result.boxes.xyxy.cpu().numpy() if len(result.boxes) > 0 else np.array([])
                    confidences = result.boxes.conf.cpu().numpy() if len(result.boxes) > 0 else np.array([])
                    class_ids = result.boxes.cls.cpu().numpy() if len(result.boxes) > 0 else np.array([])
                    
                    detection_result = DetectionResult(
                        frame=frame,
                        boxes=boxes,
                        confidences=confidences,
                        class_ids=class_ids,
                        timestamp=timestamp,
                        inference_time=inference_time
                    )
                    
                    # 출력 큐가 차면 가장 오래된 결과 제거
                    if self.output_queue.qsize() > 1:
                        try:
                            self.output_queue.get_nowait()
                        except:
                            pass
                    
                    self.output_queue.put(detection_result)
                    
                except Exception as e:
                    logger.error(f"Inference error: {e}")
                    
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
    
    def stop(self):
        self.running = False


class DisplayThread(threading.Thread):
    """결과를 시각화하고 표시하는 스레드"""
    
    def __init__(self, result_queue: Queue):
        super().__init__(daemon=True)
        self.result_queue = result_queue
        self.running = False
        self.fps_history = deque(maxlen=30)
        
    def run(self):
        """검출 결과 시각화"""
        self.running = True
        logger.info("Display thread started")
        
        try:
            while self.running:
                try:
                    result = self.result_queue.get(timeout=1)
                    
                    # FPS 계산
                    fps = 1.0 / (result.inference_time + 0.001)
                    self.fps_history.append(fps)
                    avg_fps = np.mean(self.fps_history)
                    
                    # 바운딩 박스 그리기
                    frame = result.frame.copy()
                    for box, conf, class_id in zip(result.boxes, result.confidences, result.class_ids):
                        x1, y1, x2, y2 = map(int, box)
                        label = f"{int(class_id)} {conf:.2f}"
                        
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(frame, label, (x1, y1 - 10),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    # FPS, 감지 수 표시
                    cv2.putText(frame, f"FPS: {avg_fps:.1f}", (10, 30),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(frame, f"Detections: {len(result.boxes)}", (10, 60),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    
                    cv2.imshow("YOLOv8 Detection", frame)
                    
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        self.running = False
                        
                except Exception as e:
                    logger.error(f"Display error: {e}")
                    
        finally:
            cv2.destroyAllWindows()
            self.running = False
    
    def stop(self):
        self.running = False


class YOLOv8Pipeline:
    """전체 파이프라인 관리"""
    
    def __init__(self, model_path: str = "yolov8n.pt", resolution: Tuple[int, int] = (640, 480)):
        self.camera_queue = Queue(maxsize=2)
        self.inference_queue = Queue(maxsize=2)
        
        self.camera_thread = CameraThread(self.camera_queue, resolution)
        self.inference_thread = InferenceThread(self.camera_queue, self.inference_queue, model_path)
        self.display_thread = DisplayThread(self.inference_queue)
        
    def run(self):
        """파이프라인 시작"""
        logger.info("Starting YOLOv8 pipeline...")
        
        self.camera_thread.start()
        self.inference_thread.start()
        self.display_thread.start()
        
        try:
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            logger.info("Shutting down...")
            self.stop()
    
    def stop(self):
        """파이프라인 종료"""
        self.camera_thread.stop()
        self.inference_thread.stop()
        self.display_thread.stop()
        
        self.camera_thread.join(timeout=2)
        self.inference_thread.join(timeout=2)
        self.display_thread.join(timeout=2)


if __name__ == "__main__":
    # 사용 예시
    pipeline = YOLOv8Pipeline(
        model_path="yolov8n.pt",  # nano 모델 (가장 가볍음)
        resolution=(640, 480)
    )
    pipeline.run()