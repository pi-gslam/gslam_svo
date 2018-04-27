# GSLAM-SVO

## 1. Introduction

This code is the [SVO](https://github.com/uzh-rpg/rpg_svo) plugin implementation base on [GSLAM](https://github.com/zdzhaoyong/GSLAM).

![GSLAM-SVO](./data/images/gslam_svo.gif)

## 2. Build and Install
### 2.1. Build and Install GSLAM

git clone https://github.com/zdzhaoyong/GSLAM --branch 2.4.2

### 2.2. Build and Install GSLAM-SVO

```
mkdir build;
cd build;
cmake .. -DCMAKE_BUILD_TYPE=Release;
make;
sudo make install
```

## 3. Run SVO with gslam

```
gslam SLAM=libgslam_svo Dataset=your_dataset 
```
