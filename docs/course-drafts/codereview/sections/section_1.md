## 第 1 章 Code Review 在流程中的位置

公司规定，工期超过三天的开发任务都要做 code review [00:10](https://www.bilibili.com/video/BV1Dh4y1J7iG/?t=10)。这个环节固定卡在开发结束、准备提测之前 [00:55](https://www.bilibili.com/video/BV1Dh4y1J7iG/?t=55)。

很多团队赶进度时都有一条潜规则：代码没必要写得太复杂，能跑就行。发布上线以后，代码质量不行、运行性能很差；程序员离职后，接手的人完全看不懂；本来只想改一个很小的 bug，结果无端弄出三个 bug，效率非常低 [00:22](https://www.bilibili.com/video/BV1Dh4y1J7iG/?t=22)。这些坑都源于跳过了 code review。

它的定义很朴素：

> 对别人写的代码做审核，从而确保开发质量的一种手段 [00:45](https://www.bilibili.com/video/BV1Dh4y1J7iG/?t=45)。

它是项目质量管控当中非常重要的一个环节。

### 它排在哪一步

开发人员拿到一个需求，先做设计——这个需求打算怎么做。设计本身也要评审，技术设计没问题了才开始写代码。小需求一个人做，中型或者大型的需求需要一个小组合力开发。技术设计的大方向定了，实现思路上仍可能有差别，code review 就能在这时候发现一些问题 [01:02](https://www.bilibili.com/video/BV1Dh4y1J7iG/?t=62)。

```text
需求 → 设计（设计也要评审）→ 写代码 → code review → 提测
```

放在提测之前做，收益来自提测阶段的性质。提测阶段不该是改代码的阶段，只该是完善代码的阶段：

> 提测阶段不应该在大量的去修改代码，而是去完善代码 [01:33](https://www.bilibili.com/video/BV1Dh4y1J7iG/?t=93)。

> [!NOTE]
> 提测阶段要动的是 QA 提出来的 bug，以及兼容一些极端问题、边界问题。code review 同时也在确认：提测之前所有需求功能点都已经实现，代码逻辑符合业务逻辑 [01:37](https://www.bilibili.com/video/BV1Dh4y1J7iG/?t=97)。

### 谁参加

![视频里的标题卡：需要什么人参加？](images/fig-card-roles.png)

参与者有三类。代码的开发者自己要在场。还要找技术能力强一点的同事，比如开发组长、架构师。QA 也一起拉上 [01:55](https://www.bilibili.com/video/BV1Dh4y1J7iG/?t=115)。

有人会问，QA 不写代码，为什么还要拉上 QA 参加测试。其实有必要，因为很多 QA 同学也会写代码。他们在听你讲的时候会思考：

> 你这样做是不是符合产品的需求？他那边设计的一些测试 case 是否有遗漏？ [02:18](https://www.bilibili.com/video/BV1Dh4y1J7iG/?t=138)

QA 在场不是陪坐。review 结束时要确认的东西里，有一部分只能由懂业务的人问出来。

