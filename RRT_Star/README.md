# Rapidly-Exploring Random Tree Star(RRT*)

RRT* (Rapidly-exploring Random Tree Star) is an advanced version of the RRT (Rapidly-exploring Random Tree) algorithm.  The core of this algorithm consists of two parts: **reselecting the parent node** and **rewiring**.

算法详解：[Rapidly-Exploring Random Tree Star](https://blog.csdn.net/weixin_51995147/article/details/143838991?sharetype=blogdetail&sharerId=143838991&sharerefer=PC&sharesource=weixin_51995147&spm=1011.2480.3001.8118)

## 1.CPP

**Build**

```bash
cd PathPlanning/RRT_Star/CPP
mkdir build && cd build
cmake ..
make
```

**Run**

In the "build" directory

```bash
./rrtstar
```

![](https://s2.loli.net/2024/11/17/4zQopeWRMhkaIJd.png)

## 2.Python

**Run**

```bash
cd PathPlanning/RRT_Star/Python
python demo.py
```

![](https://s2.loli.net/2024/11/17/7SOwDZXtLrevQHp.gif)