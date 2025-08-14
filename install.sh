#!/bin/sh
# ROS 2 (Humble/Foxy) 환경을 위한 의존성 설치 스크립트

# 스크립트 실행 중 오류 발생 시 즉시 중단
set -e

# --- 섹션 1: 시스템 및 Python 의존성 설치 ---
echo ">>> 1. 기본 시스템 및 Python 의존성을 설치합니다."
sudo apt-get update
sudo apt-get install -y python3-pip
pip3 install --user transforms3d networkx pygame
sudo apt-get install -y libeigen3-dev

# --- 섹션 2: ROS 2 의존성 설치 (rosdep 사용 권장) ---
echo ">>> 2. ROS 2 의존성 설치를 확인합니다."
echo ">>> 이 스크립트 실행 전, 작업 공간 루트에서 'rosdep install'을 먼저 실행하는 것이 좋습니다."

# --- 섹션 3: OSQP 및 OsqpEigen 소스 클론 및 시스템 설치 ---
echo ">>> 3. 최적화 솔버(OSQP, OsqpEigen)를 클론하고 빌드하여 시스템에 설치합니다."

# 임시 빌드를 위한 폴더 생성
BUILD_DIR=$(mktemp -d)
echo ">>> 임시 빌드 폴더: $BUILD_DIR"
cd $BUILD_DIR

# OSQP 클론 (사용자님의 기존 레포지토리 주소로 수정)
echo ">>> OSQP 클론 중..."
OSQP_repo="https://github.com/rise-lab-skku-racing/osqp.git" # <<< 주소를 원래대로 수정했습니다.
git clone --recurse-submodules "$OSQP_repo" ./osqp
cd osqp
mkdir build && cd build
cmake ..
make -j$(nproc)
echo ">>> OSQP를 시스템에 설치합니다. (sudo 비밀번호 필요)"
sudo make install
cd ../../

# OsqpEigen 클론
echo ">>> OsqpEigen 클론 중..."
OSQP_eigen_repo="https://github.com/robotology/osqp-eigen.git"
git clone --recurse-submodules "$OSQP_eigen_repo" ./osqp-eigen
cd osqp-eigen
mkdir build && cd build
cmake ..
make -j$(nproc)
echo ">>> OsqpEigen을 시스템에 설치합니다. (sudo 비밀번호 필요)"
sudo make install
cd ../../

# 임시 빌드 폴더 삭제
echo ">>> 임시 빌드 폴더를 정리합니다."
rm -rf $BUILD_DIR
