# D Star(Dynamic A Star) Algorithm

D Star algorithm is an algorithm used in dynamic environments to quickly update path as the environment where obstacles can change over time. D Star algorithm traditionally works in a **backward manner**. It starts planning from the goal to the start, as this allows for more efficient updates when obstacles are detected near the robot's current position.

算法详解：https://blog.csdn.net/weixin_51995147/article/details/142097627?spm=1001.2014.3001.5502

## 1.CPP

### Build

```bash
cd PathPlanning/D_Star/CPP
mkdir build && cd build
cmake ..
make
```

### Run

In the "build" directory

```bash
./dstar
```

![](https://s2.loli.net/2024/09/10/3CT7ejaIAx1bcht.png)

## 2.Python

Python>=3.10 is required.

### Run

```bash
cd PathPlanning/D_Star/Python
python demo.py
```

![](https://s2.loli.net/2024/09/10/NWeyfAT4xm13Xqb.gif)