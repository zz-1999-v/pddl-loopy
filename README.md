### 安装pddlstream环境
[pddlstream](https://github.com/caelan/pddlstream/tree/stable)

### 替换文件
* 将loopy2文件夹放入`/examples/pybullet/`目录下
* 将utlis文件夹中的`utlis/pybullet_tools/loopy_primitives`加入pddlstream项目同目录下
* 将同目录下utlis中的`models`文件与`utlis.py`文件进行替换

### 执行代码
`python -m examples.pybullet.loopy.run -teleport`
