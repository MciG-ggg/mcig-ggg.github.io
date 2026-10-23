---
title: 计算智能 模拟卷
timestamp: 2026-10-23T18:00:00+08:00
series: 计算智能
tags: [计算智能, 进化算法, GA, ES, DE, PSO, NSGA-II, 约束处理, 期末复习, 模拟卷]
description: 《计算智能》期末模拟卷三套，每套一百分。题型是判断、填空、编程、大题。编程题给的是完整实现，大题是概念计算题和算法流程模拟题，每处结论都链回课件对应页。
source_url: http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/
toc: true
draft: false
---

```cardlink
url: http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/
title: "计算智能 · 课程课件"
description: "中南大学自动化学院《计算智能》课程的十二个课件"
host: 10.188.31.55:5244
```

三套卷，每套一百分，题型是**判断、填空、编程、大题**。**答案在每道题右上角的「显示答案」按钮后面。** 做完一整卷再一次性展开对答案，也可以一题一题看。

**编程题**给的是完整实现，不是片段。照着抄一遍就能跑，注意每一步的边界处理、随机下标的抽法和强制维。

**大题**就是概念计算题和算法流程模拟题——不给复杂函数让你求导，而是让你按算法的每一步手动推演一两次算子，判断每一步的输入、输出和依据。完整课程笔记在[《计算智能：进化算法与优化方法》](/lecture/计算智能_进化算法与优化方法)。

老师的备考备忘录在 wiki 上：[代码速通](https://wiki.grtsinry43.com/zh/computing-intelligence/code)、[计算智能计算题](https://wiki.grtsinry43.com/zh/computing-intelligence/calculate)、[计算智能重点](https://wiki.grtsinry43.com/zh/computing-intelligence/important)、[计算智能知识框架](https://wiki.grtsinry43.com/zh/computing-intelligence/structure)。

| 卷 | 范围 | 判断 | 填空 | 编程 | 大题 | 合计 |
|---|---|---|---|---|---|---|
| 第一卷 | 思想框架与公平比较 | 12 分 | 16 分 | 32 分 | 40 分 | 100 分 |
| 第二卷 | 约束处理、多目标与混合整数 | 10 分 | 20 分 | 30 分 | 40 分 | 100 分 |
| 第三卷 | 算法流程模拟 | 10 分 | 20 分 | 30 分 | 40 分 | 100 分 |

## 第一卷 · 思想框架与公平比较

### 答题卡

| 题号 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 得分 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 类型 | 判 | 判 | 判 | 判 | 判 | 判 | 填 | 填 | 填 | 填 | 编 | 大 | 大 | |
| 满分 | 2 | 2 | 2 | 2 | 2 | 2 | 4 | 4 | 4 | 4 | 32 | 20 | 20 | 100 |

### 一、判断题

::::quiz{title="第一卷"}
:::q{type="判断" points="2" answer="2"}
进化算法能保证找到问题的全局最优解。

---

**错误。**

进化算法**只在概率意义上**给出强搜索，不提供最优性保证。给定有限代数 $G_{max}$ 与有限种群 $N$，搜索范围天然有限；多峰地形上「离最优还差很远但曲线已经平了」是常态 [Chapter 1_ Introduction to EAs.pdf · p.2](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%201_%20Introduction%20to%20EAs.pdf#page=2)。

它的价值在于**对函数几乎不做假设、不需要梯度、在合理时间内给出足够好的解**，不在于「保证最优」。

:::

:::q{type="判断" points="2" answer="1"}
目标函数 $f$ 默认越小越好，适应度默认越大越好，两者方向相反。

---

**正确。**

最小化问题里常常取 $\mathrm{fit} = -f$，正是因为方向相反必须换算。GA 的课件表里 $f$ 与 $\mathrm{fit}$ 是分两列的，正是为了提醒这件事 [Chapter 1_ Introduction to EAs.pdf · p.8](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%201_%20Introduction%20to%20EAs.pdf#page=8)。

注意这只是常见做法，不是普适规则：有的问题里 $\mathrm{fit}$ 是 $f$ 的某个非线性变换。

:::

:::q{type="判断" points="2" answer="1"}
比较两个进化算法的性能时，应该固定最大适应度评估次数 $FES_{max}$，而不是固定迭代代数。

---

**正确。**

$FES_{max}$ 是最重要的公平性原则，它代表算法可用的总计算预算。不同算法每代所需的评估次数可能不同，固定代数就等于给了它们不同的预算 [Chapter 7_ Algorithm Comparison.pdf · p.1](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%207_%20Algorithm%20Comparison.pdf#page=1)。

:::

:::q{type="判断" points="2" answer="2"}
种群 $P(t)$ 和父代集 $S(t)$ 是同一个东西，只是叫法不同。

---

**错误。**

种群 $P(t)$ 是**当前一代的全部** $N$ 个个体；父代集 $S(t)$ 是经选择算子挑出来的那**一小批当父母的个体**，规模 $N_p$ 通常 $\le N$ [Chapter 1_ Introduction to EAs.pdf · p.7](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%201_%20Introduction%20to%20EAs.pdf#page=7)。

框架图里「种群」框和「父代集」框必须是两个独立的框，中间有「选择」箭头。ES 更极端：它把父代个数写作 $\mu$、每代子代数写作 $\lambda$，且 $\lambda \ge \mu$、常用 $\lambda \approx 7\mu$，父代集比子代还少。

:::

:::q{type="判断" points="2" answer="1"}
变异是探索的主要来源，交叉是开发的核心。

---

**正确。**

变异把个体朝新方向拽一拽，是探索的主要来源；交叉把已有优良解附近的部件重新组合，是开发的核心 [Chapter 1_ Introduction to EAs.pdf · p.3](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%201_%20Introduction%20to%20EAs.pdf#page=3)。

交叉这一步**不评价函数值**，只是重组已有信息。

:::

:::q{type="判断" points="2" answer="2"}
收敛曲线已经变平但停在很差的局部最优上，应该把选择的严格度调高，让强者垄断繁殖权。

---

**错误。**

早熟收敛说明多样性已经丢光，算法太偏向开发、忘了探索。选择的严格度要**调低**——选择太严，适应度高的那一两个个体几乎垄断繁殖权，其他个体全被边缘化，多样性迅速丢光 [Chapter 1_ Introduction to EAs.pdf · p.3](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%201_%20Introduction%20to%20EAs.pdf#page=3)。

:::

### 二、填空题

:::q{type="填空" points="4" answer="选择|替换"}
进化算法通用框架的六步闭环是：初始化、计算适应度、{{blank}}、交叉、变异、{{blank}}。

---

**答案：选择、替换。**

完整六步是：初始化 → 计算适应度 → 选择 → 交叉 → 变异 → 替换，然后判断终止条件，不满足就 $t \leftarrow t+1$ 回到第二步 [Chapter 1_ Introduction to EAs.pdf · p.7](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%201_%20Introduction%20to%20EAs.pdf#page=7)。

四问决定它是哪个算法：选择怎么做、交叉怎么做、变异怎么做、替换怎么做。

:::

:::q{type="填空" points="4" answer="d(H)|o(H)"}
模式定理里，交叉项用的是模式 $H$ 的定义长度 {{blank}}，变异项用的是它的阶 {{blank}}。

---

**答案：$d(H)$、$o(H)$。**

问「哪个量决定交叉存活概率」答 $d(H)$，问「哪个量决定变异存活概率」答 $o(H)$。两者数学上完全不同：定义长度是这段模式里能被交换的**位置数**，阶是这段模式里**1 的个数** [Chapter 2_ Introduction to GAs.pdf · p.15](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%202_%20Introduction%20to%20GAs.pdf#page=15)。

:::

:::q{type="填空" points="4" answer="最大适应度评估次数|Wilcoxon 秩和检验"}
公平比较时，除了算法本身以外其他条件都要一致。其中最重要的原则是固定 {{blank}}；单次运行有偶然性，要在每个测试函数上独立运行多次，并用 {{blank}} 判断两者的差异是否显著。

---

**答案：最大适应度评估次数 $FES_{max}$、Wilcoxon 秩和检验。**

不能简单地说平均值小的就一定更好。课件要求用统计检验来判断差异是否在统计学上显著 [Chapter 7_ Algorithm Comparison.pdf · p.1](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%207_%20Algorithm%20Comparison.pdf#page=1)。

:::

:::q{type="填空" points="4" answer="调高|调高|调低"}
早熟收敛时三个旋钮的调整方向：变异强度 {{blank}}、种群规模 $N$ {{blank}}、选择的严格度 {{blank}}。

---

**答案：调高、调高、调低。**

三个旋钮都朝「更偏探索」的方向调，但具体方向不同：变异是探索的主要来源，调高；更多个体意味着同时盯着更多山谷，调高；选择太严会让强者垄断繁殖权、多样性迅速丢光，所以调低 [Chapter 1_ Introduction to EAs.pdf · p.3](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%201_%20Introduction%20to%20EAs.pdf#page=3)。

:::

### 三、编程题

:::q{type="编程" points="32"}
用 Python 实现 DE/rand/1/bin 最小化 Sphere 函数 $f(x) = \sum_i x_i^2$，$x_i \in [-10, 10]$。要求写出变异、二项式交叉、一对一贪婪选择三步，处理好边界，并写一个多次独立运行的实验函数。

---

**参考实现**（照抄能跑，注释是得分点）：

```python
import numpy as np

# --- 1. 问题定义 ---
def sphere_function(x):
    """Sphere 测试函数，全局最优解在 x=[0,0,...,0]，最小值为 0。"""
    return np.sum(x ** 2)


# --- 2. 算法参数设置 ---
D = 10          # 变量维度 (Dimension)
NP = 50         # 种群大小 (Population Size)
F = 0.5         # 缩放因子 (Scaling Factor)
CR = 0.9        # 交叉概率 (Crossover Rate)
MAX_GEN = 1000  # 最大迭代代数
LOWER_BOUND = -10
UPPER_BOUND = 10


# --- 3. 核心算法实现 ---
def de_rand_1_bin():
    # 初始化种群：在 [LOWER_BOUND, UPPER_BOUND] 内随机生成 NP x D 的矩阵
    population = LOWER_BOUND + (UPPER_BOUND - LOWER_BOUND) * np.random.rand(NP, D)

    # 评估初始种群的适应度
    fitness = np.array([sphere_function(ind) for ind in population])

    for g in range(MAX_GEN):
        for i in range(NP):
            # --- a. 变异 (Mutation) ---
            # 从种群中随机选三个与当前个体 i 不同的个体
            idxs = [idx for idx in range(NP) if idx != i]
            r1, r2, r3 = np.random.choice(idxs, 3, replace=False)

            # v_i = x_r1 + F * (x_r2 - x_r3)
            mutant_vector = population[r1] + F * (population[r2] - population[r3])

            # 边界处理：确保变异向量在搜索空间内
            mutant_vector = np.clip(mutant_vector, LOWER_BOUND, UPPER_BOUND)

            # --- b. 交叉 (Crossover) ---
            trial_vector = np.copy(population[i])
            # 至少有一维必须来自变异向量，否则试验向量就等于目标向量
            j_rand = np.random.randint(0, D)
            for j in range(D):
                if np.random.rand() < CR or j == j_rand:
                    trial_vector[j] = mutant_vector[j]

            # --- c. 选择 (Selection) ---
            # 一对一贪婪选择：试验向量更好就替换
            trial_fitness = sphere_function(trial_vector)
            if trial_fitness < fitness[i]:
                population[i] = trial_vector
                fitness[i] = trial_fitness

    best_idx = np.argmin(fitness)
    return population[best_idx], fitness[best_idx]


# --- 4. 实验设置与运行（体现"公平比较"）---
def run_experiment(num_runs=30):
    """多次独立运行并统计，这是"公平比较"的实践部分。"""
    results = []
    for i in range(num_runs):
        _, best_fitness = de_rand_1_bin()
        results.append(best_fitness)
        print(f"Run {i+1}/{num_runs}, Best Fitness Found: {best_fitness}")

    results = np.array(results)
    print("\n--- Experiment Results ---")
    print(f"Best: {np.min(results)}")
    print(f"Worst: {np.max(results)}")
    print(f"Mean: {np.mean(results)}")
    print(f"Std Dev: {np.std(results)}")


if __name__ == "__main__":
    run_experiment(num_runs=10)
```

**四个容易丢分的细节：**

1. **下标必须互不相同且不等于 $i$。** `np.random.choice(idxs, 3, replace=False)` 里 `idxs` 已经排除了 $i$，`replace=False` 保证三个下标互不相同 [Chapter 4_ Introduction to Differential Evolution.pdf · p.6](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%204_%20Introduction%20to%20Differential%20Evolution.pdf#page=6)。
2. **边界处理。** 变异向量可能飞出 $[-10,10]$，要 `np.clip`。不处理会让目标函数算在界外，破坏比较的前提。
3. **强制维 $j_{rand}$。** 二项式交叉里必须保证至少一维来自变异向量，否则试验向量完全等于目标向量，交叉这一步等于没做 [Chapter 4_ Introduction to Differential Evolution.pdf · p.7](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%204_%20Introduction%20to%20Differential%20Evolution.pdf#page=7)。
4. **多次独立运行 + 统计。** 即使题目只要求跑一次，也要写 `run_experiment`，这体现你懂得随机算法必须做多次运行和统计。

:::

### 四、大题

:::q{type="解答" points="20"}
请阐述进化算法的基本思想和通用框架，并说明探索与开发的平衡。

---

**基本思想。** 核心源于达尔文的自然选择和适者生存理论。把问题的潜在解看作一个种群 (Population) 中的个体 (Individual)，通过模拟生物进化中的选择 (Selection)、交叉 (Crossover) 和变异 (Mutation)，使种群一代代地向更优的方向进化，最终找到最优解或近似最优解。

**通用框架（六步闭环）：**

1. **初始化**：在搜索空间里随机生成初始种群 $P(0)$，这一步只做一次。
2. **计算适应度**：对每个个体算 $\mathrm{fit}(x_i)$，准备给选择用。最小化问题里常取 $\mathrm{fit} = -f$。
3. **选择**：按适应度从 $P(t)$ 里挑出 $N_p$ 个个体组成**父代集** $S(t)$。产物是父代集，它不等于 $P(t)$。
4. **交叉**：把 $S(t)$ 里的父代配对，交换零件，生成候选子代 $C(t)$。这一步不评价函数值。
5. **变异**：对 $C(t)$ 里每个个体做一次小幅随机扰动，得到 $M(t)$。作用是维持多样性、防止过早收敛。
6. **替换**：从 $P(t) \cup M(t)$（或仅 $M(t)$）里挑出 $N$ 个组成 $P(t+1)$。关键问题是老的要不要保留。

**终止判断**：满足终止条件（达到最大代数、目标值足够小、连续若干代没改进）就输出当前最优个体，否则 $t \leftarrow t+1$ 回到第二步。

**探索与开发的平衡。** 探索是去没去过的地方找新的谷，代价是可能白跑；开发是在已知不错的点附近细挖，代价是可能永远困在这一个谷里。变异和多样性维持主要靠探索，选择压力和交叉主要靠开发。过于偏重探索会难以收敛，过于偏重开发容易陷入局部最优 [Chapter 1_ Introduction to EAs.pdf · p.3](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%201_%20Introduction%20to%20EAs.pdf#page=3)。

一个优秀的进化算法必须在这两者之间取得平衡。这就是「凡是问某参数调大或调小会怎样，答案几乎都能这样开头」的具体体现。

:::

:::q{type="解答" points="20"}
在比较两种不同进化算法的性能时，应遵循哪些原则来确保公平性？

---

**一、统一的测试平台。** 必须在**相同的一组标准测试函数**上比较。这些函数要有不同特性（单模/多模、可分/不可分、高维/低维），才能全面考察算法性能 [Chapter 7_ Algorithm Comparison.pdf · p.1](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%207_%20Algorithm%20Comparison.pdf#page=1)。

**二、统一的计算资源。** 固定**最大适应度评估次数** $FES_{max}$，而不是固定迭代代数——不同算法每代所需的评估次数可能不同，固定代数等于给了它们不同的预算。这是最重要的公平性原则。

**三、消除随机性影响。** 进化算法有随机性，单次运行结果具有偶然性。必须在每个测试函数上独立运行算法多次（课件提到 25 次或 50 次），再做统计分析。

**四、一致的性能指标。** 用相同的指标衡量结果。单目标问题通常比较多���运行的最优解的**平均值**和**标准差**。

**五、科学的统计分析。** 不能简单地说平均值小的就一定更好。应该用统计检验（如 Wilcoxon 秩和检验）判断两者的性能差异是否在统计学上显著。

**六、说明比较的到底是代数还是函数评价预算。** 同一个算法改一下 $\lambda$ 就能在相同代数下产生完全不同的试探次数，不说清楚就没有可比性 [Chapter 3_ Introduction to ES and EP.pdf · p.4](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%203_%20Introduction%20to%20ES%20and%20EP.pdf#page=4)。

:::

:::
:::

## 第二卷 · 约束处理、多目标与混合整数

### 答题卡

| 题号 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 得分 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 类型 | 判 | 判 | 判 | 判 | 判 | 填 | 填 | 填 | 填 | 填 | 编 | 大 | 大 | |
| 满分 | 2 | 2 | 2 | 2 | 2 | 4 | 4 | 4 | 4 | 4 | 30 | 20 | 20 | 100 |

### 一、判断题

::::quiz{title="第二卷"}
:::q{type="判断" points="2" answer="2"}
罚函数法里罚因子设得越大越好，因为惩罚越强，越能逼算法找到可行解。

---

**错误。**

罚因子太小，对不可行解的惩罚不足，算法可能最终收敛到**不可行解**；罚因子太大，会过度惩罚那些虽然不可行但可能包含通往最优解有用信息的「潜力股」个体，导致算法过早收敛到某个次优的可行解 [Chapter 9_ Evolutionary Algorithms for Solving Single-objective Constrained Optimization Problems.pdf · p.4](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%209_Evolutionary%20Algorithms%20for%20Solving%20Single-objective%20Constrained%20Optimization%20Problems.pdf#page=4)。

罚因子难以设定，这是罚函数法的致命缺点。

:::

:::q{type="判断" points="2" answer="1"}
Deb 的可行性准则：可行解永远优于不可行解，即使不可行解的目标函数值更小。

---

**正确。**

三条规则的第一条：可行解 $G(A)=0$ 优于不可行解 $G(B)>0$，此时**不比较目标函数值** [Chapter 9_ Evolutionary Algorithms for Solving Single-objective Constrained Optimization Problems.pdf · p.4](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%209_Evolutionary%20Algorithms%20for%20Solving%20Single-objective%20Constrained%20Optimization%20Problems.pdf#page=4)。

:::

:::q{type="判断" points="2" answer="1"}
NSGA-II 里，同一层内每个目标维度上取最大和最小的那两个个体，拥挤度设为无穷大。

---

**正确。**

边界点的拥挤度设为 $\infty$，以确保它们被保留，扩大帕累托前沿的范围 [Chapter 8_ Evolutionary Algorithms for Solving Multiobjective Optimization Problems.pdf · p.13](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%208_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Optimization%20Problems.pdf#page=13)。

:::

:::q{type="判断" points="2" answer="2"}
NSGA-II 的选择策略是先比拥挤度，再比等级。

---

**错误。**

顺序反了。NSGA-II 是**先比等级，再比密度**：先通过非支配排序保证精英解（等级高的）被保留，再通过拥挤度距离保证在精英解内部的多样性 [Chapter 8_ Evolutionary Algorithms for Solving Multiobjective Optimization Problems.pdf · p.13](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%208_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Optimization%20Problems.pdf#page=13)。

:::

:::q{type="判断" points="2" answer="1"}
CaR 框架的切割策略，是把目标值差于 $f_{best}$ 的空间区域变成不可行域。

---

**正确。**

切割是在每一代动态地添加约束 $f(x) \le f_{best}$，把目标值大于 $f_{best}$ 的区域「切割」掉，削减搜索空间、引导算法专注于更有希望的区域。

:::

### 二、填空题

:::q{type="填空" points="4" answer="可行解|约束违反度|目标函数值"}
Deb 的可行性准则三条规则：{{blank}} 优于不可行解；都不可行时比 {{blank}}；都可行时比 {{blank}}。

---

**答案：可行解、约束违反度、目标函数值。**

三条规则按优先级排：可行解优于不可行解 → 都不可行则违反度小的获胜 → 都可行则目标函数值更优的获胜 [Chapter 9_ Evolutionary Algorithms for Solving Single-objective Constrained Optimization Problems.pdf · p.4](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%209_Evolutionary%20Algorithms%20for%20Solving%20Single-objective%20Constrained%20Optimization%20Problems.pdf#page=4)。

它不需要任何参数，逻辑清晰，实现简单，但过于严格地偏好可行解，可能丢弃一些包含有用信息的不可行解。

:::

:::q{type="填空" points="4" answer="收敛性|多样性"}
NSGA-II 的两个核心机制：快速非支配排序保证 {{blank}}，拥挤度距离保证 {{blank}}。

---

**答案：收敛性、多样性。**

非支配排序给个体分层，Rank 1 永远优于 Rank 2，强烈的选择压力驱动种群向真实前沿**收敛**；拥挤度距离在同一层内优先保留处于稀疏区域的个体，让解在前沿上**均匀分布** [Chapter 8_ Evolutionary Algorithms for Solving Multiobjective Optimization Problems.pdf · p.13](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%208_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Optimization%20Problems.pdf#page=13)。

:::

:::q{type="填空" points="4" answer="小于等于|严格小于"}
帕累托支配的定义：A 支配 B 的条件是，A 在所有目标上的值都 {{blank}} B 的对应值，并且至少有一个目标上 {{blank}} B。

---

**答案：小于等于、严格小于。**

「都不比 B 差，并且至少在一个目标上严格优于 B」。如果两个目标上的值都相等，则互不支配 [Chapter 8_ Evolutionary Algorithms for Solving Multiobjective Optimization Problems.pdf · p.8](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%208_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Optimization%20Problems.pdf#page=8)。

:::

:::q{type="填空" points="4" answer="切割|排斥"}
CaR 框架针对混合整数规划的两个策略是 {{blank}} 和 {{blank}}。

---

**答案：切割、排斥。**

切割提高搜索效率，排斥保证搜索的全局性。排斥的做法是：连续 $T$ 代没有找到更好的解时，判定算法停滞在一个可行块里，对整数部分与已探索块相同的解加一个巨大的约束违反度惩罚，逼算法离开 [Chapter 10_ Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.10](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=10)。

:::

:::q{type="填空" points="4" answer="罚因子"}
罚函数法最致命的缺点是 {{blank}} 难以设定。

---

**答案：罚因子。**

罚因子太小可能收敛到不可行解，罚因子太大会把「潜力股」全灭掉、过早收敛到次优可行解。这是罚函数法唯一但致命的软肋 [Chapter 9_ Evolutionary Algorithms for Solving Single-objective Constrained Optimization Problems.pdf · p.4](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%209_Evolutionary%20Algorithms%20for%20Solving%20Single-objective%20Constrained%20Optimization%20Problems.pdf#page=4)。

:::

### 三、编程题

:::q{type="编程" points="30"}
用 Python 实现 NSGA-II 的两个核心算子：快速非支配排序（fast non-dominated sort）和拥挤度距离（crowding distance）。输入是一群解的目标函数值矩阵，输出每个解的 rank 和拥挤度。

---

**参考实现**：

```python
import numpy as np


def dominates(a, b):
    """a 支配 b：a 的每个目标都不大于 b，且至少有一个目标严格小于。

    a、b 都是目标函数值向量。最小化问题。
    """
    return bool(np.all(a <= b) and np.any(a < b))


def fast_non_dominated_sort(F):
    """快速非支配排序。

    F: (N, m) 的目标函数值矩阵，N 个个体 m 个目标。
    返回 (rank, fronts)：
      rank[i] 是第 i 个个体的层号，从 0 开始（0 即第一层）；
      fronts 是层的列表，每层装着该层的个体下标。
    """
    n = F.shape[0]
    S = [[] for _ in range(n)]        # S[p]：被 p 支配的个体
    N = np.zeros(n, dtype=int)        # N[p]：支配 p 的个体数
    rank = np.zeros(n, dtype=int)
    fronts = [[]]

    # 第一步：两两比较，算出谁支配谁
    for p in range(n):
        for q in range(n):
            if p == q:
                continue
            if dominates(F[p], F[q]):
                S[p].append(q)        # p 支配 q
            elif dominates(F[q], F[p]):
                N[p] += 1             # q 支配 p
        if N[p] == 0:                  # 没被任何个体支配
            rank[p] = 0
            fronts[0].append(p)

    # 第二步：逐层剥离
    i = 0
    while fronts[i]:
        next_front = []
        for p in fronts[i]:
            for q in S[p]:
                N[q] -= 1
                if N[q] == 0:          # q 的支配者都处理完了
                    rank[q] = i + 1
                    next_front.append(q)
        i += 1
        fronts.append(next_front)

    return rank, fronts


def crowding_distance(F, front):
    """拥挤度距离：同一层内，邻居越远说明周围越空旷，拥挤度越大。

    F: (N, m) 目标函数值矩阵。
    front: 该层个体的下标列表。
    返回长度 N 的数组，只有该层个体的位置被填上，其余为 0。
    """
    n = len(front)
    dist = np.zeros(F.shape[0])

    if n <= 2:
        # 少于三个个体时无法算距离，全部视为无穷大，保证都被保留
        for idx in front:
            dist[idx] = np.inf
        return dist

    for m in range(F.shape[1]):
        # 按第 m 个目标排序
        order = np.argsort(F[front, m])
        f_min = F[front[order[0]], m]
        f_max = F[front[order[-1]], m]
        rng = f_max - f_min

        # 该维度上的两个边界个体设为无穷大
        dist[front[order[0]]] = np.inf
        dist[front[order[-1]]] = np.inf

        if rng == 0:
            # 这个目标上所有个体一样，取不出差异，跳过
            continue

        # 中间个体：两侧邻居的差，除以该维度的跨度
        for k in range(1, n - 1):
            prev_idx = front[order[k - 1]]
            next_idx = front[order[k + 1]]
            dist[front[order[k]]] += (F[next_idx, m] - F[prev_idx, m]) / rng

    return dist
```

**三个容易丢分的细节：**

1. **支配的判据是「都不大于」加「至少一个严格小于」。** 只写「都小于」会把互不支配的解错判成支配 [Chapter 8_ Evolutionary Algorithms for Solving Multiobjective Optimization Problems.pdf · p.8](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%208_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Optimization%20Problems.pdf#page=8)。
2. **边界个体的拥挤度是 $\infty$，不是 0。** 这是为了让前沿两端不丢失。
3. **每个目标维度各排一次序，累加。** 拥挤度是所有维度上「邻居间距的归一化差值」之和，单个维度算一次是不够的。

:::

### 四、大题

:::q{type="解答" points="20"}
请阐述至少三种主流的进化算法约束处理技术，并分析它们的优缺点。

---

**背景。** 单目标约束优化问题 (SCOP) 要同时最小化 $f(x)$ 并让 $x$ 满足一组约束。除了目标函数值，还要度量一个解违反了约束多少，叫**约束违反度** $G(x)$：不等式约束那支取 $\max\{0, g_j(x)\}$，等式约束那支取 $\max\{0, |h_j(x)| - \delta\}$（$\delta$ 是正容差）。违反度为零的那片区域是可行域 [Chapter 9_ Evolutionary Algorithms for Solving Single-objective Constrained Optimization Problems.pdf · p.4](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%209_Evolutionary%20Algorithms%20for%20Solving%20Single-objective%20Constrained%20Optimization%20Problems.pdf#page=4)。

**技术一：罚函数法。** 把有约束问题转成无约束问题，构造 $F(x) = f(x) + P(G(x))$，不可行时惩罚项为正，可行时为 0。

*优点*：思想简单，易于实现。
*缺点*：**罚因子难以设定**。太小则惩罚不足，算法可能收敛到不可行解；太大则过度惩罚那些虽不可行却可能包含通往最优解有用信息的「潜力股」个体，导致过早收敛到次优可行解。

**技术二：可行性准则（Deb's Feasibility Rule）。** 比较两个个体时**不混合**目标函数和违反度，而是按优先级决策：可行解优于不可行解 → 都不可行则违反度小的获胜 → 都可行则目标函数值更优的获胜。

*优点*：无需任何参数，逻辑清晰，实现简单，在大多数问题上表现稳健。
*缺点*：过于严格地偏好可行解，可能丢弃一些包含有用信息的不可行解。

**技术三：多目标优化法。** 把原问题直接看成一个双目标问题：目标 1 是 $\min f(x)$，目标 2 是 $\min G(x)$，然后直接套用成熟的多目标优化算法（如 NSGA-II）求解。

*优点*：把权衡过程显式化，无需设定罚因子，能自然保留那些目标函数值优秀但轻微违反约束的「潜力股」个体。解是 f 与 G 之间的帕累托前沿，其中 $G(x)=0$ 的那个点就是我们想要的。
*缺点*：计算开销相对较大，因为要做非支配排序等操作。

:::

:::q{type="解答" points="20"}
什么是帕累托支配？请详细阐述 NSGA-II 算法如何利用非支配排序和拥挤度距离来寻找帕累托最优解集。

---

**一、帕累托支配。** 对一个同时最小化 $m$ 个目标的问题，如果个体 A 在所有目标上都不比个体 B 差，并且至少在一个目标上严格优于 B，那么称 **A 支配 B**。如果一个解在种群中不被任何其他解所支配，它就是一个**帕累托最优解（非支配解）**。所有帕累托最优解在目标函数空间中的集合是**帕累托前沿**。算法的目标是找到一组尽可能逼近并均匀覆盖真实前沿的解 [Chapter 8_ Evolutionary Algorithms for Solving Multiobjective Optimization Problems.pdf · p.8](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%208_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Optimization%20Problems.pdf#page=8)。

**二、快速非支配排序（保证收敛性）。** 目的是给种群中的个体划分「等级」，优先选择更靠近真实前沿的解。

过程是：找出种群中所有非支配解，标记为第一层 (Rank 1)；暂时忽略第一层，在剩余的解中再次找出所有非支配解，标记为第二层；如此重复直到所有个体都被分层。

作用：选择时 Rank 1 的个体永远优于 Rank 2 的个体。这种强烈的选择压力驱动种群整体向帕累托前沿收敛。

**三、拥挤度距离（保证多样性）。** 目的是在同一层内优先选择处于「稀疏」区域的个体，避免解都挤在一起。

过程是：同一层内，针对每个目标维度分别排序；该层在该维度上最大和最小的两个个体（边界点）拥挤度设为无穷大；中间每个个体的拥挤度是其两侧邻居在该维度上差值之和（通常再除以该维度的跨度做归一化）。直观上邻居离得越远说明它周围越空旷。

作用：两个个体 Rank 相同时，拥挤度更大的获胜。这鼓励解在整个前沿上均匀分布。

**四、一句话概括。** NSGA-II 的选择策略是**先比等级，再比密度**。先通过非支配排序保证精英解被保留，再通过拥挤度距离保证精英解内部的多样性 [Chapter 8_ Evolutionary Algorithms for Solving Multiobjective Optimization Problems.pdf · p.13](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%208_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Optimization%20Problems.pdf#page=13)。

:::

:::
:::

## 第三卷 · 算法流程模拟

### 答题卡

| 题号 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 得分 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 类型 | 判 | 判 | 判 | 判 | 判 | 填 | 填 | 填 | 填 | 填 | 编 | 编 | 大 | 大 | |
| 满分 | 2 | 2 | 2 | 2 | 2 | 4 | 4 | 4 | 4 | 4 | 15 | 15 | 20 | 20 | 100 |

### 一、判断题

::::quiz{title="第三卷"}
:::q{type="判断" points="2" answer="2"}
DE/rand/1/bin 的基向量是当前最优个体。

---

**错误。**

DE/rand/1/bin 的基向量是**随机挑**的 $x_{r1,G}$；用当前最优做基向量的是 **DE/best/1/bin** [Chapter 4_ Introduction to Differential Evolution.pdf · p.18](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%204_%20Introduction%20to%20Differential%20Evolution.pdf#page=18)。

DE/best/1/bin 收敛更快但多样性更低、容易早熟；DE/rand/1/bin 探索更强、慢一些但不易卡死。

:::

:::q{type="判断" points="2" answer="1"}
DE 的选择步骤是：试验向量比目标向量好就替换，否则保留目标向量。

---

**正确。**

这是一对一贪婪选择。最小化问题里比的是 $f$，试验向量更小就替换 [Chapter 4_ Introduction to Differential Evolution.pdf · p.5](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%204_%20Introduction%20to%20Differential%20Evolution.pdf#page=5)。

:::

:::q{type="判断" points="2" answer="2"}
$(1{+}1)$-ES 是逗号策略，子代不参与父代的竞争。

---

**错误。**

$(1{+}1)$-ES 是**加号策略**：父代和子代一起进选择池，比完留下好的，父代有优先权（精英保留）。逗号策略 $(\mu,\lambda)$ 才是每代淘汰全部父代、只从子代里选 [Chapter 3_ Introduction to ES and EP.pdf · p.4](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%203_%20Introduction%20to%20ES%20and%20EP.pdf#page=4)。

:::

:::q{type="判断" points="2" answer="1"}
多峰地形上应该用逗号策略 $(\mu,\lambda)$，因为它允许暂时退化。

---

**正确。**

单峰问题用加号 $(\mu{+}\lambda)$——精英保留、收敛快；多峰或崎岖地形用逗号 $(\mu,\lambda)$——允许暂时变差、能逃出局部最优 [Chapter 3_ Introduction to ES and EP.pdf · p.4](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%203_%20Introduction%20to%20ES%20and%20EP.pdf#page=4)。

问「哪个允许退化」答 $(\mu,\lambda)$，问「多峰用哪个」也答 $(\mu,\lambda)$。

:::

:::q{type="判断" points="2" answer="1"}
PSO 的速度由旧速度、个体历史最优和全局最优三项合成。

---

**正确。**

$$
v_{i}^{t+1} = w \cdot v_{i}^{t} + c_1 r_1 \odot (p_{best,i} - x_{i}^{t}) + c_2 r_2 \odot (g_{best} - x_{i}^{t})
$$

$w$ 是惯性权重，$c_1$ 是「相信自己的经验」，$c_2$ 是「相信种群意见」[Chapter 5_ Introduction to Particle Swarm Optimization.pdf · p.17](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%205_%20Introduction%20to%20Particle%20Swarm%20Optimization.pdf#page=17)。

PSO 没有选择算子、没有交叉、没有变异，粒子的永不淘汰。

:::

### 二、填空题

:::q{type="填空" points="4" answer="0.4|2"}
PSO 的速度更新里，惯性权重 $w$ 常用 0.9 线性递减到 {{blank}}，加速常量 $c_1$ 与 $c_2$ 常取 {{blank}}。

---

**答案：0.4、2。**

$w$ 大 = 冲得猛（偏探索），$w$ 小 = 稳下来（偏开发）。课件给的标准配置是 $w$ 从 0.9 线性递减到 0.4，$c_1 = c_2 = 2$ [Chapter 5_ Introduction to Particle Swarm Optimization.pdf · p.4](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%205_%20Introduction%20to%20Particle%20Swarm%20Optimization.pdf#page=4)。

:::

:::q{type="填空" points="4" answer="16|0.625"}
GA 的轮盘赌：种群 4 个个体，适应度 $\{2, 4, 4, 6\}$。适应度总和是 {{blank}}，第三个个体的累积概率区间的右端点是 {{blank}}（左端点是 0.375）。

---

**答案：16、0.625。**

$p = [2/16, 4/16, 4/16, 6/16] = [0.125, 0.25, 0.25, 0.375]$，累积概率 $q = [0.125, 0.375, 0.625, 1.000]$。随机数 $r = 0.45$ 落在 $[0.375, 0.625)$，选中第三个个体 [Chapter 2_ Introduction to GAs.pdf · p.6](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%202_%20Introduction%20to%20GAs.pdf#page=6)。

:::

:::q{type="填空" points="4" answer="τ|1"}
$(1{+}1)$-ES 的步长自适应公式是 $\delta' = \delta \cdot \exp(\tau \cdot N(0,1))$，指数里的系数记作 {{blank}}；取 $D = 1$ 时这个系数等于 {{blank}}。

---

**答案：$\tau$、1。**

$\tau = (\sqrt{D})^{-1}$，$D=1$ 时 $\tau = 1$，公式简化为 $\delta' = \delta \exp(N(0,1))$ [Chapter 3_ Introduction to ES and EP.pdf · p.6](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%203_%20Introduction%20to%20ES%20and%20EP.pdf#page=6)。

步长也是个体的一部分：子代被拒绝时，步长要和解一起回退。

:::

:::q{type="填空" points="4" answer="强制维"}
DE 的二项式交叉里，试验向量至少有一维必须来自变异向量，这一维叫 {{blank}}，它的下标是 $j_{rand}$。

---

**答案：强制维。**

没有强制维的话，当所有维的随机数都大于 $CR$ 时，试验向量会完全等于目标向量，交叉这一步等于没做 [Chapter 4_ Introduction to Differential Evolution.pdf · p.7](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%204_%20Introduction%20to%20Differential%20Evolution.pdf#page=7)。

:::

:::q{type="填空" points="4" answer="1/L"}
二进制 GA 里，变异概率 $p_m$ 通常取 {{blank}} 附近（$L$ 是二进制串长度），$p_c$ 通常取 0.6 到 0.9。

---

**答案：$1/L$。**

$p_m$ 统计的是「每一位翻不翻」，$L=10$ 时 $p_m \approx 0.05$，$L=100$ 时 $p_m \approx 0.01$，平均一条串翻不到一位。搞混量级会让 GA 退化成纯随机搜索（$p_m$ 太大），或让变异形同虚设（$p_m$ 太小，易早熟收敛）[Chapter 2_ Introduction to GAs.pdf · p.8](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%202_%20Introduction%20to%20GAs.pdf#page=8)。

:::

### 三、编程题

:::q{type="程序" points="15"}
实现遗传算法的一代：轮盘赌选择、单点交叉、位翻转变异，跑 $N_{gen}$ 代后输出最优解。

---

**参考实现**：

```python
import numpy as np

# --- 参数 ---
DNA_SIZE = 24          # DNA 长度，24 位二进制
POP_SIZE = 200         # 种群大小
CROSSOVER_RATE = 0.8   # 交叉概率
MUTATION_RATE = 0.005  # 变异概率
N_GENERATIONS = 50     # 迭代次数
X_BOUND = [-3, 3]      # x 的取值范围


def F(x):
    """适应度函数（这里是一个待最大化的函数）。"""
    return np.sin(10 * x) * x + np.cos(2 * x) * x


def get_fitness(pred):
    """把可能为负的值平移到正数，保证轮盘赌的概率非负。"""
    return pred + 1e-3 - np.min(pred)


def translateDNA(pop):
    """二进制 DNA 转成十进制的 x。"""
    return pop.dot(2 ** np.arange(DNA_SIZE)[::-1]) / float(2 ** DNA_SIZE - 1) * (X_BOUND[1] - X_BOUND[0]) + X_BOUND[0]


def select(pop, fitness):
    """轮盘赌选择：按适应度占比有放回地抽 POP_SIZE 个个体。"""
    idx = np.random.choice(np.arange(POP_SIZE), size=POP_SIZE, replace=True, p=fitness / fitness.sum())
    return pop[idx]


def crossover(parent, other):
    """单点交叉：从随机位置切开，后半段换成另一个父代的。"""
    cross_point = np.random.randint(low=0, high=DNA_SIZE)
    child = parent.copy()
    child[cross_point:] = other[cross_point:]
    return child


def mutation(child):
    """位翻转变异：以 MUTATION_RATE 的概率翻转一位。"""
    if np.random.rand() < MUTATION_RATE:
        mutate_point = np.random.randint(0, DNA_SIZE)
        child[mutate_point] = child[mutate_point] ^ 1
    return child


def evolve(pop, fitness):
    """跑一代：选择 -> 交叉 -> 变异。"""
    new_pop = []
    for parent in pop:
        child = parent.copy()
        if np.random.rand() < CROSSOVER_RATE:
            other = pop[np.random.randint(POP_SIZE)]
            child = crossover(child, other)
        child = mutation(child)
        new_pop.append(child)
    return np.array(new_pop)


def main():
    pop = np.random.randint(2, size=(POP_SIZE, DNA_SIZE))
    for _ in range(N_GENERATIONS):
        fitness = get_fitness(F(translateDNA(pop)))
        pop = select(pop, fitness)
        pop = evolve(pop, fitness)
    fitness = get_fitness(F(translateDNA(pop)))
    best = pop[np.argmax(fitness)]
    print("最优解：", translateDNA(best))
    print("最大适应度：", F(translateDNA(best)))


if __name__ == "__main__":
    main()
```

**细节：**

1. **适应度要先平移。** `get_fitness` 里加 `1e-3 - np.min(pred)`，保证概率非负，且不会因为全是 0 而除零。
2. **变异是异或。** `child[mutate_point] ^ 1` 翻转一位，不是置 0 或置 1。
3. **交叉点随机且可以取到 DNA_SIZE。** `np.random.randint(low=0, high=DNA_SIZE)` 取值范围是 $[0, DNA\_SIZE)$，等于 DNA_SIZE 时相当于不换后半段，也合法。

:::

:::q{type="程序" points="15"}
实现粒子群优化：速度更新、位置更新与边界处理、个体历史最优与全局最优的维护。

---

**参考实现**：

```python
import numpy as np

# --- 参数 ---
bounds = [(-513, 513), (-513, 513)]   # 每一维的取值范围
particle_count = 100
iterations = 100
W = 0.7        # 惯性权重
C1 = 1.4       # 认知分量
C2 = 1.4       # 社会分量


def f(v):
    """目标函数（这里是最小化）。"""
    return -(v[1] + 47) * np.sin(np.sqrt(np.abs(v[1] + (v[0] / 2) + 47))) - v[0] * np.sin(
        np.sqrt(np.abs(v[0] - v[1] - 47))
    )


def initialize_particle(bounds):
    """初始化一个粒子：位置在范围内随机，速度在 [-1, 1] 内随机。"""
    position = np.random.uniform(bounds[0], bounds[1], len(bounds))
    velocity = np.random.uniform(-1, 1, len(bounds))
    return position, velocity


def update_velocity(position, velocity, best_position, global_best):
    """速度更新：旧速度 + 认知分量 + 社会分量。"""
    r1, r2 = np.random.rand(2, len(velocity))
    cognitive_velocity = C1 * r1 * (best_position - position)
    social_velocity = C2 * r2 * (global_best - position)
    return W * velocity + cognitive_velocity + social_velocity


def update_position(position, velocity, bounds):
    """位置更新：位置加上速度，并做边界处理（修补算子）。"""
    position = position + velocity
    return np.clip(position, bounds[0], bounds[1])


def update_global_best(best_positions, global_best):
    """全局最优：取所有个体历史最优里最好的那个。"""
    current_best = max(best_positions, key=f)
    if f(current_best) < f(global_best):
        global_best = current_best
    return global_best


def particle_swarm_optimization(f, particle_count, bounds, iterations):
    particles = [initialize_particle(bounds) for _ in range(particle_count)]
    best_positions = [p[0].copy() for p in particles]
    global_best = min(best_positions, key=f).copy()

    for _ in range(iterations):
        for i in range(particle_count):
            position, velocity = particles[i]
            velocity = update_velocity(position, velocity, best_positions[i], global_best)
            position = update_position(position, velocity, bounds)
            particles[i] = (position, velocity)
            # 个体历史最优：只有当前位置更优才更新
            if f(position) < f(best_positions[i]):
                best_positions[i] = position.copy()
        global_best = update_global_best(best_positions, global_best)

    return global_best


if __name__ == "__main__":
    result = particle_swarm_optimization(f, particle_count, bounds, iterations)
    print("最优解：", result)
    print("目标函数值：", f(result))
```

**细节：**

1. **PSO 没有选择、交叉、变异。** 粒子永不淘汰，「好起来」靠的是它记着 `pbest`。
2. **个体历史最优只在当前位置更优时才更新。** 当前解变差时 `pbest` 不回退 [Chapter 5_ Introduction to Particle Swarm Optimization.pdf · p.14](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%205_%20Introduction%20to%20Particle%20Swarm%20Optimization.pdf#page=14)。
3. **边界处理用 `np.clip`（修补算子）。** 位置飞出区间要拽回来，否则速度和位置会一路发散。
4. **`best_positions` 要拷贝。** 用 `position.copy()`，否则 `pbest` 和粒子共用一块内存，粒子一动 `pbest` 就跟着变，历史最优就没了。

:::

### 四、大题

:::q{type="解答" points="20"}
在二维最小化问题中，当前目标个体 $x_i = (2, 5)$。种群中随机选择了三个其他个体：$x_{r1} = (1, 8)$、$x_{r2} = (10, 3)$、$x_{r3} = (4, 4)$。算法参数为 $F = 0.5$、$CR = 0.8$。请模拟 DE/rand/1/bin 策略的一步，算出最终进入下一代的个体。

---

**第一步：变异。**

$$
v_i = x_{r1} + F \cdot (x_{r2} - x_{r3})
$$

代入：

$$
v_i = (1, 8) + 0.5 \cdot \big((10, 3) - (4, 4)\big) = (1, 8) + 0.5 \cdot (6, -1) = (1, 8) + (3, -0.5) = (4, 7.5)
$$

**变异向量 $v_i = (4, 7.5)$。**

**第二步：二项式交叉。**

生成试验向量 $u_i$，每一维要么来自 $v_i$，要么来自 $x_i$。设每维的随机数是 $rand_1 = 0.7$、$rand_2 = 0.9$：

- 第 1 维：$0.7 \le CR = 0.8$，条件成立，$u_i[0]$ 取自变异向量，$u_i[0] = 4$。
- 第 2 维：$0.9 > CR = 0.8$，条件不成立，$u_i[1]$ 取自目标向量，$u_i[1] = 5$。

**试验向量 $u_i = (4, 5)$。**

（如果某维恰好被选成强制维 $j_{rand}$，不管随机数多大都取自变异向量。）

**第三步：选择。**

设目标函数 $f(x) = x_1^2 + x_2^2$：

$$
f(x_i) = f(2, 5) = 4 + 25 = 29, \qquad f(u_i) = f(4, 5) = 16 + 25 = 41
$$

最小化问题里 $29 < 41$，原目标向量更优，**保留 $x_i$**。

**最终答案：经过一轮 DE/rand/1/bin 操作后，进入下一代的个体仍然是 $x_i = (2, 5)$。**

**答题时必须写清的三件事：每一步的公式、每一步的输入输出、选择时比较的是适应度而不是目标值本身。** 即使算错小数，过程分也能拿到。

:::

:::q{type="解答" points="20"}
一个最小化双目标问题（目标为 $f_1$ 和 $f_2$），当前种群有 6 个解：A(0.1, 0.9)、B(0.2, 0.7)、C(0.4, 0.4)、D(0.5, 0.2)、E(0.3, 0.8)、F(0.6, 0.5)。(1) 做非支配排序，指出每一层。(2) 计算第一层所有解的拥挤度距离。

---

**第一问：非支配排序。**

核心原则：最小化问题中 A 支配 B 的条件是 A 的所有目标值都小于等于 B，且至少一个目标值严格小于 B。用「淘汰法」逐层找。

**找第一层（Rank 1）：**

- A(0.1, 0.9)：B、C、D、E、F 的 $f_1$ 都比 A 差、$f_2$ 都比 A 好 → 互不支配。**A 是第一层。**
- B(0.2, 0.7)：A 的 $f_1$ 比 B 好、$f_2$ 比 B 差；C、D、E、F 的 $f_1$ 都比 B 差 → 互不支配。**B 是第一层。**
- C(0.4, 0.4)：B 的 $f_2$ 比 C 差，A 的 $f_1$ 比 C 好但不全面；D、E、F 的 $f_1$ 都比 C 差 → **C 是第一层。**
- D(0.5, 0.2)：其余解的 $f_1$ 都比 D 差 → **D 是第一层。**
- E(0.3, 0.8)：B(0.2, 0.7) 的 $f_1$ 和 $f_2$ 都严格小于 E → **B 支配 E，E 不是第一层。**
- F(0.6, 0.5)：C(0.4, 0.4) 的 $f_1$ 和 $f_2$ 都严格小于 F → **C 支配 F，F 不是第一层。**

**第一层（Rank 1）= {A, B, C, D}。**

**找第二层（Rank 2）：** 剩下 E(0.3, 0.8) 和 F(0.6, 0.5)。E 的 $f_1$ 优于 F，$f_2$ 劣于 F → 互不支配。

**第二层（Rank 2）= {E, F}。**

**第二问：拥挤度距离。**

个体 $i$ 在目标 $m$ 上的拥挤度是它两侧邻居在该目标上的差值除以该目标的跨度；个体总拥挤度是所有目标上的拥挤度之和。只对第一层 {A, B, C, D} 计算。

初始化：$CD(A) = CD(B) = CD(C) = CD(D) = 0$。

**按第一个目标 $f_1$ 排序：** A(0.1)、B(0.2)、C(0.4)、D(0.5)。边界点 A 和 D 的拥挤度设为 $\infty$。$f_1$ 跨度 $= 0.5 - 0.1 = 0.4$。

- $CD(B)$ 的 $f_1$ 部分 $= (0.4 - 0.1)/0.4 = 0.75$
- $CD(C)$ 的 $f_1$ 部分 $= (0.5 - 0.2)/0.4 = 0.75$

**按第二个目标 $f_2$ 排序：** D(0.2)、C(0.4)、B(0.7)、A(0.9)。边界点 D 和 A 继续是 $\infty$。$f_2$ 跨度 $= 0.9 - 0.2 = 0.7$。

- $CD(C)$ 的 $f_2$ 部分 $= (0.7 - 0.2)/0.7 \approx 0.714$
- $CD(B)$ 的 $f_2$ 部分 $= (0.9 - 0.4)/0.7 \approx 0.714$

**累加：**

$$
CD(A) = \infty, \qquad CD(D) = \infty, \qquad CD(B) = CD(C) = 0.75 + 0.714 \approx 1.464
$$

**最终答案：** $CD(A) = \infty$，$CD(B) = 1.464$，$CD(C) = 1.464$，$CD(D) = \infty$。

**考试技巧：** 在卷面上清晰地写出排序过程和计算公式，即使算错小数，过程分也能拿到。**关键是别把两个目标的排序搞混**——每个目标各排一次，累加。

:::

:::
:::
