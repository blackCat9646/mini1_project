# Mini Project Rules

이 문서는 `mini1_ws` 프로젝트의 코드 작성 기준입니다.
앞으로 노드, 런치, 설정, 문서를 만들거나 수정할 때 이 기준을 우선합니다.

## 기본 목표

- 초등학생도 흐름을 따라갈 수 있는 쉬운 구조
- 발표자가 그대로 설명할 수 있는 코드와 주석
- 필요한 기능만 담은 작은 파일
- ROS 2 방식에 맞는 노드, 토픽, 서비스, 액션 분리
- 공식 문서 우선 확인

## 코드 스타일

- 코드 한 파일에 모든 기능을 몰아넣지 않기
- 노드 하나는 역할 하나만 담당하기
- 변수명은 의미가 바로 보이게 작성하기
- 복잡한 추상화보다 읽기 쉬운 함수 우선 사용
- 숨겨진 동작보다 명확한 조건문 우선 사용
- 안전 관련 로직은 짧고 분명하게 작성

## 주석 규칙

- 주석은 한국어로 작성
- 주석은 명사형으로 작성
- 주석은 발표용 컨닝페이퍼처럼 작성
- 코드 한 줄을 그대로 번역하는 주석 금지
- 판단 기준, 좌표 기준, 안전 기준은 반드시 주석 작성

좋은 예:

```python
# RC카 감지 실패 시 정지 명령
```

나쁜 예:

```python
# 정지한다
```

좋은 예:

```python
# 목표 거리보다 멀 때 전진 속도 계산
```

나쁜 예:

```python
# linear.x 설정
```

## 시스템 기준

- OS: Ubuntu 22.04
- ROS 2: Humble
- Robot: TurtleBot 4 계열
- Navigation: Nav2
- Robot RGB-D camera: OAK-D
- Robot LiDAR: RPLIDAR
- Fixed camera: USB webcam, depth 없음
- Map: `/home/rokey/Documents/student_maps/hoon_map.yaml`

## 센서 역할

- USB webcam: 특정 감시 구역에서 RC카 등장 여부 확인
- OAK-D RGB: 로봇 시점에서 RC카 탐지 또는 추적
- OAK-D depth: RC카까지 거리 계산
- RPLIDAR: 앞쪽 장애물 안전 확인
- Nav2: 관찰 지점까지 자율 이동

## 현재 설계 방향

- USB webcam으로 RC카를 정확한 map 좌표로 변환하지 않음
- USB webcam은 RC카 등장 trigger 역할
- RC카가 감지되면 미리 정한 관찰 waypoint로 이동
- 관찰 waypoint 도착 후 OAK-D로 RC카 직접 탐색
- OAK-D depth와 RPLIDAR를 함께 사용해 안전 추종

## YOLO 기준

- 1차 구현: COCO pretrained YOLO의 `car` 클래스 사용
- RC카가 불안정하게 잡히면 LabelImg로 직접 데이터 수집
- 직접 학습 시 클래스 이름은 `rc_car` 사용 검토
- 고정 webcam 단계는 detect trigger 중심
- OAK-D 추종 단계는 필요 시 ByteTrack 기반 tracking 사용

## ROS 2 통신 기준

- Topic: 계속 흐르는 센서와 결과 데이터
- Service: 시작, 정지, 상태 변경 같은 짧은 요청
- Action: Nav2 이동처럼 오래 걸리는 작업
- TF2: 카메라, 로봇, 맵 좌표 관계
- Parameter: 속도, 거리, confidence 같은 조절값
- YAML: waypoint, 노드 설정, 학습 데이터 설정

## 공식 문서 우선순위

문서나 구현 판단이 애매하면 아래 공식 문서를 우선 확인합니다.

- TurtleBot 4 manual
- TurtleBot 4 Waffle / platform manual, 사용 환경에 맞는 항목
- ROS 2 Humble documentation
- Nav2 documentation
- OAK-D / DepthAI documentation
- RPLIDAR 관련 공식 또는 제조사 문서
- Ultralytics YOLO documentation

## 안전 기준

- RC카를 못 보면 정지
- depth 값이 이상하면 정지
- LiDAR 앞쪽이 위험하면 정지
- Nav2 이동 실패 시 무리하게 계속 전진하지 않기
- 첫 실제 주행은 낮은 속도에서 테스트
- `/cmd_vel` 발행 노드는 항상 timeout 조건 포함
