#!/usr/bin/env python3
"""
Lepton 열화상 이미지 시각화 프로그램
PGM 파일을 읽어서 OpenCV로 표시
"""

import cv2
import numpy as np
import glob
import os
import argparse

def read_pgm(filename):
    """
    PGM (P2) 파일을 읽어서 numpy 배열로 반환
    
    Args:
        filename: PGM 파일 경로
    
    Returns:
        numpy array: 이미지 데이터
    """
    with open(filename, 'r') as f:
        # P2 헤더 읽기
        header = f.readline().strip()
        if header != 'P2':
            raise ValueError(f"지원하지 않는 형식: {header} (P2만 지원)")
        
        # 주석 건너뛰기
        line = f.readline().strip()
        while line.startswith('#'):
            line = f.readline().strip()
        
        # 이미지 크기 읽기
        width, height = map(int, line.split())
        
        # 최대값 읽기
        maxval = int(f.readline().strip())
        
        # 이미지 데이터 읽기
        data = []
        for line in f:
            data.extend([int(x) for x in line.split()])
        
        # numpy 배열로 변환 (60x80)
        img = np.array(data, dtype=np.uint16).reshape(height, width)
        
        return img, maxval

def visualize_thermal(img, colormap=cv2.COLORMAP_JET):
    """
    열화상 이미지를 컬러맵으로 시각화
    
    Args:
        img: 원본 이미지 (numpy array)
        colormap: OpenCV 컬러맵
    
    Returns:
        컬러 이미지
    """
    # 0-255 범위로 정규화
    normalized = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX)
    normalized = normalized.astype(np.uint8)
    
    # 컬러맵 적용
    colored = cv2.applyColorMap(normalized, colormap)
    
    # 크기 확대 (60x80 -> 480x640, 8배)
    scaled = cv2.resize(colored, (640, 480), interpolation=cv2.INTER_NEAREST)
    
    return scaled, normalized

def main():
    parser = argparse.ArgumentParser(description='Lepton 열화상 이미지 뷰어')
    parser.add_argument('-f', '--file', type=str, help='특정 PGM 파일 경로')
    parser.add_argument('-d', '--directory', type=str, default='.', 
                       help='PGM 파일이 있는 디렉토리 (기본값: 현재 디렉토리)')
    parser.add_argument('-c', '--colormap', type=str, default='JET',
                       choices=['JET', 'HOT', 'RAINBOW', 'OCEAN', 'TURBO'],
                       help='컬러맵 선택 (기본값: JET)')
    
    args = parser.parse_args()
    
    # 컬러맵 매핑
    colormap_dict = {
        'JET': cv2.COLORMAP_JET,
        'HOT': cv2.COLORMAP_HOT,
        'RAINBOW': cv2.COLORMAP_RAINBOW,
        'OCEAN': cv2.COLORMAP_OCEAN,
        'TURBO': cv2.COLORMAP_TURBO
    }
    colormap = colormap_dict[args.colormap]
    
    # 파일 목록 가져오기
    if args.file:
        files = [args.file]
    else:
        pattern = os.path.join(args.directory, 'IMG_*.pgm')
        files = sorted(glob.glob(pattern))
        
        if not files:
            print(f"PGM 파일을 찾을 수 없습니다: {pattern}")
            return
        
        print(f"발견된 파일: {len(files)}개")
    
    current_idx = 0
    
    while True:
        if current_idx < 0:
            current_idx = 0
        elif current_idx >= len(files):
            current_idx = len(files) - 1
        
        filename = files[current_idx]
        
        try:
            # PGM 파일 읽기
            img, maxval = read_pgm(filename)
            
            # 시각화
            colored, normalized = visualize_thermal(img, colormap)
            
            # 정보 표시
            info_text = [
                f"File: {os.path.basename(filename)} ({current_idx+1}/{len(files)})",
                f"Size: {img.shape[1]}x{img.shape[0]}",
                f"Range: {img.min()} - {img.max()} (max: {maxval})",
                f"Colormap: {args.colormap}",
                "",
                "Keys: [n]ext [p]rev [q]uit [s]ave [c]olormap"
            ]
            
            y_offset = 30
            for text in info_text:
                cv2.putText(colored, text, (10, y_offset), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                y_offset += 25
            
            # 화면 표시
            cv2.imshow('Lepton Thermal Image', colored)
            
            # 키 입력 처리
            key = cv2.waitKey(0) & 0xFF
            
            if key == ord('q') or key == 27:  # q 또는 ESC
                break
            elif key == ord('n'):  # 다음 이미지
                current_idx += 1
            elif key == ord('p'):  # 이전 이미지
                current_idx -= 1
            elif key == ord('s'):  # 현재 화면 저장
                save_name = f"thermal_vis_{current_idx:04d}.png"
                cv2.imwrite(save_name, colored)
                print(f"저장됨: {save_name}")
            elif key == ord('c'):  # 컬러맵 변경
                colormap_list = list(colormap_dict.keys())
                current_cm_idx = colormap_list.index(args.colormap)
                next_cm_idx = (current_cm_idx + 1) % len(colormap_list)
                args.colormap = colormap_list[next_cm_idx]
                colormap = colormap_dict[args.colormap]
                print(f"컬러맵 변경: {args.colormap}")
                
        except Exception as e:
            print(f"파일 읽기 오류 ({filename}): {e}")
            current_idx += 1
            if current_idx >= len(files):
                break
    
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()