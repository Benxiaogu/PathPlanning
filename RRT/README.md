# Rapidly-Exploring Random Tree (RRT)

A Rapidly-Exploring Random Tree is an algorithm based on random sampling used for path planning. The tree is constructed incrementally from samples drawn randomly from the search space and is inherently biased to grow towards large unsearched areas of the problem.

算法详解：https://blog.csdn.net/weixin_51995147/article/details/142034688?spm=1001.2014.3001.5502

## 1.CPP

### Build

```bash
cd PathPlanning/RRT/CPP
mkdir build && cd build
cmake ..
make
```

### Run

In the "build" directory

```bash
./rrt
```

The result for rrt :

![](https://s2.loli.net/2024/09/08/7xTqu8EU4lRHhAY.png)

## 2.Python

Python>=3.10 is required.

### Run

```bash
cd PathPlanning/RRT/Python
python demo.py
```

The result for rrt :

![](https://s2.loli.net/2024/09/08/SNoz8UGqBbOY4Cv.gif)