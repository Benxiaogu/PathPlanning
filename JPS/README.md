# Jump Point Search(JPS)

Jump Point Search (JPS) is an optimized pathfinding algorithm that enhances the performance of the A* algorithm on grid-based maps, particularly in regular grids like four-way or eight-way grids. JPS accelerates the search process by skipping over redundant nodes, thereby reducing the number of nodes that need to be expanded.

算法详解：https://blog.csdn.net/weixin_51995147/article/details/141351237?spm=1001.2014.3001.5502

## 1.CPP

### Build

```bash
cd PathPlanning/JPS/CPP
mkdir build && cd build
cmake ..
make
```

### Run

In the 'build' directory

```bash
./jps
```

![](https://s2.loli.net/2024/08/20/IGFA3JgwLlyXhNj.png)

## 2.Python

Python>=3.10 required

### Run

```bash
cd PathPlanning/JPS/Python
python demo.py
```

![](https://s2.loli.net/2024/08/20/HVMJQ9FAmUluGP2.gif)

