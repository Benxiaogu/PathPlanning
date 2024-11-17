# RRT-connect: An efficient approach to single-query path planning

RRT-Connect , which is a variation of RRT, grows two trees from both the source and destination until they meet and grows the trees towards each other (rather then towards random direction), the last but not least is the greediness becomes stronger by growing the tree with multiple epsilon steps instead of a single one.

The method is proposed in paper [RRT-connect: An efficient approach to single-query path planning](https://www.cs.cmu.edu/afs/cs/academic/class/15494-s12/readings/kuffner_icra2000.pdf).

The code for this algorithm is as follows:

![](https://s2.loli.net/2024/11/14/Ypqa3RNd9yxkH2s.png)

## 1.CPP

**Build**

```bash
cd PathPlanning/RRT-Connect/CPP
mkdir build && cd build
cmake ..
make
```

**Run**

In the "build" directory

```bash
./rrtconnect
```

The result for rrtconnect:

![](https://s2.loli.net/2024/11/14/Q7fz9y6EADNtjZk.png)

## 2.Python

**Run**

```bash
cd PathPlanning/RRT-Connect/Python
python demo.py
```

The result for rrtconnect:

![](https://s2.loli.net/2024/11/14/yheqQk2gR4f7X5o.gif)