### 安装pddlstream环境
[pddlstream](https://github.com/caelan/pddlstream/tree/stable)

### 替换文件
* 将loopy2文件夹放入`/examples/pybullet/`目录下
* 将utils文件夹中的`utils/pybullet_tools/loopy_primitives`加入pddlstream项目同目录下
* 将同目录下utils中的`models`文件与`utils.py`文件进行替换

### 执行代码
`python -m examples.pybullet.loopy.run -teleport`
